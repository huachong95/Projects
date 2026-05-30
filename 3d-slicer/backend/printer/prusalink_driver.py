"""PrusaLink REST API driver for Prusa MK4 (and other PrusaLink-enabled printers).

Reference: https://github.com/prusa3d/Prusa-Link-Web (spec/openapi.yaml)

Notes about the real PrusaLink (Buddy firmware) API that this driver depends on:
- Auth: the API key shown on the printer's PrusaLink screen is sent as the
  ``X-Api-Key`` header. (HTTP Digest auth with user "maker" is the documented
  alternative; the header is what MK4/MINI accept and what users have.)
- Status:  GET /api/v1/status  ->  {"printer": {...}, "job": {...}}
- Job:     GET /api/v1/job     ->  current job incl. id + file name
- Control: PUT /api/v1/job/{id}/pause | /resume  ;  DELETE /api/v1/job/{id}
- Upload:  PUT /api/v1/files/{storage}/{path}  (header Print-After-Upload: ?1)
- Print:   POST /api/v1/files/{storage}/{path}
- Camera:  GET /api/v1/cameras/snap  (JPEG snapshot; no MJPEG stream exists)
- There is intentionally NO arbitrary G-code endpoint in the PrusaLink v1 API.
"""
import asyncio
import time
from enum import Enum
from typing import Optional

import httpx
from pydantic import BaseModel


class PrinterState(str, Enum):
    IDLE = "IDLE"
    BUSY = "BUSY"
    READY = "READY"
    PRINTING = "PRINTING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"
    STOPPED = "STOPPED"
    ATTENTION = "ATTENTION"
    ERROR = "ERROR"
    UNKNOWN = "UNKNOWN"


class PrinterStatus(BaseModel):
    state: PrinterState = PrinterState.UNKNOWN
    temp_hotend: float = 0.0
    temp_hotend_target: float = 0.0
    temp_bed: float = 0.0
    temp_bed_target: float = 0.0
    progress_percent: float = 0.0
    current_layer: int = 0
    total_layers: int = 0
    filename: Optional[str] = None
    eta_seconds: Optional[int] = None
    job_id: Optional[int] = None
    camera_available: bool = False
    camera_url: Optional[str] = None
    connected: bool = False


# HTTP status codes we treat as success for command-style requests.
# MK4 firmware sometimes answers 200 where the spec says 204, so accept both.
_OK = (200, 201, 204)


class PrusaLinkDriver:
    def __init__(self, host: str, api_key: str, timeout: float = 10.0):
        # Tolerate a host that already includes a scheme.
        host = host.strip().rstrip("/")
        if host.startswith("http://") or host.startswith("https://"):
            self.base_url = host
        else:
            self.base_url = f"http://{host}"
        self.headers = {"X-Api-Key": api_key}
        self.timeout = timeout
        self._client: Optional[httpx.AsyncClient] = None

        # Resolved lazily after connect.
        self._storage: Optional[str] = None
        self._camera_available = False

        # Snapshot cache so monitor polling (multiple times/sec) does not
        # hammer the printer; refreshed at most once per interval.
        self._snapshot: Optional[bytes] = None
        self._snapshot_ts: float = 0.0
        self._snapshot_min_interval = 0.8
        self._snapshot_lock = asyncio.Lock()

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #
    async def connect(self) -> bool:
        self._client = httpx.AsyncClient(
            base_url=self.base_url,
            headers=self.headers,
            timeout=self.timeout,
        )
        try:
            resp = await self._client.get("/api/v1/info")
        except httpx.RequestError:
            await self.disconnect()
            return False

        if resp.status_code == 401:
            # Wrong API key — close the client so we don't pretend to be up.
            await self.disconnect()
            return False
        if resp.status_code != 200:
            await self.disconnect()
            return False

        # Best-effort: figure out which storage to upload to and whether a
        # camera is present. Neither is fatal if it fails.
        await self._resolve_storage()
        await self._probe_camera()
        return True

    async def disconnect(self) -> None:
        if self._client:
            await self._client.aclose()
            self._client = None
        self._storage = None
        self._camera_available = False
        self._snapshot = None

    @property
    def is_connected(self) -> bool:
        return self._client is not None

    # ------------------------------------------------------------------ #
    # Status
    # ------------------------------------------------------------------ #
    async def get_status(self) -> PrinterStatus:
        if not self._client:
            return PrinterStatus()
        try:
            resp = await self._client.get("/api/v1/status")
            resp.raise_for_status()
            data = resp.json()
        except (httpx.RequestError, httpx.HTTPStatusError, ValueError):
            # Keep the connection but report a disconnected-looking status so
            # the UI degrades gracefully instead of erroring.
            return PrinterStatus(connected=True)

        printer = data.get("printer") or {}
        job = data.get("job") or {}

        state_str = str(printer.get("state", "UNKNOWN")).upper()
        try:
            state = PrinterState(state_str)
        except ValueError:
            state = PrinterState.UNKNOWN

        filename = None
        job_id = job.get("id")
        # /api/v1/status omits the file name; pull it from /api/v1/job when a
        # job is active.
        if job:
            filename = await self._get_job_filename()

        return PrinterStatus(
            state=state,
            temp_hotend=float(printer.get("temp_nozzle") or 0.0),
            temp_hotend_target=float(printer.get("target_nozzle") or 0.0),
            temp_bed=float(printer.get("temp_bed") or 0.0),
            temp_bed_target=float(printer.get("target_bed") or 0.0),
            progress_percent=float(job.get("progress") or 0.0),
            filename=filename,
            eta_seconds=job.get("time_remaining"),
            job_id=job_id,
            camera_available=self._camera_available,
            camera_url="/api/monitoring/camera/snapshot" if self._camera_available else None,
            connected=True,
        )

    async def _get_current_job_id(self) -> Optional[int]:
        if not self._client:
            return None
        try:
            resp = await self._client.get("/api/v1/job")
            if resp.status_code == 204:
                return None
            resp.raise_for_status()
            return resp.json().get("id")
        except (httpx.RequestError, httpx.HTTPStatusError, ValueError):
            return None

    async def _get_job_filename(self) -> Optional[str]:
        if not self._client:
            return None
        try:
            resp = await self._client.get("/api/v1/job")
            if resp.status_code == 204:
                return None
            resp.raise_for_status()
            file_obj = resp.json().get("file") or {}
            return file_obj.get("display_name") or file_obj.get("name")
        except (httpx.RequestError, httpx.HTTPStatusError, ValueError):
            return None

    # ------------------------------------------------------------------ #
    # Job control  (pause / resume / cancel)
    # ------------------------------------------------------------------ #
    async def pause(self) -> bool:
        return await self._job_action("pause")

    async def resume(self) -> bool:
        return await self._job_action("resume")

    async def cancel(self) -> bool:
        return await self._job_action("cancel")

    async def _job_action(self, action: str) -> bool:
        if not self._client:
            return False
        job_id = await self._get_current_job_id()
        if job_id is None:
            return False
        try:
            if action == "cancel":
                resp = await self._client.delete(f"/api/v1/job/{job_id}")
            else:
                resp = await self._client.put(f"/api/v1/job/{job_id}/{action}")
            return resp.status_code in _OK
        except httpx.RequestError:
            return False

    # ------------------------------------------------------------------ #
    # Upload + print
    # ------------------------------------------------------------------ #
    async def upload_gcode(self, gcode_path: str, filename: str,
                           print_after: bool = False) -> bool:
        if not self._client:
            return False
        storage = self._storage or "usb"
        headers = {"Content-Type": "application/octet-stream", "Overwrite": "?1"}
        if print_after:
            headers["Print-After-Upload"] = "?1"
        try:
            with open(gcode_path, "rb") as f:
                content = f.read()
            resp = await self._client.put(
                f"/api/v1/files/{storage}/{filename}",
                content=content,
                headers=headers,
            )
            return resp.status_code in _OK
        except (httpx.RequestError, OSError):
            return False

    async def start_print(self, filename: str) -> bool:
        if not self._client:
            return False
        storage = self._storage or "usb"
        try:
            resp = await self._client.post(f"/api/v1/files/{storage}/{filename}")
            return resp.status_code in _OK
        except httpx.RequestError:
            return False

    # ------------------------------------------------------------------ #
    # G-code  (NOT supported by the PrusaLink v1 API)
    # ------------------------------------------------------------------ #
    async def send_gcode(self, command: str) -> bool:
        # PrusaLink (Buddy firmware) deliberately exposes no arbitrary G-code
        # endpoint. Movement / temperature / fan control are therefore not
        # available over PrusaLink. Return False so callers report this rather
        # than silently pretending success.
        return False

    @staticmethod
    def supports_gcode() -> bool:
        return False

    # ------------------------------------------------------------------ #
    # Camera (snapshot based)
    # ------------------------------------------------------------------ #
    @property
    def camera_available(self) -> bool:
        return self._camera_available

    async def get_snapshot(self) -> Optional[bytes]:
        """Return a recent JPEG snapshot, cached briefly to avoid hammering."""
        if not self._client or not self._camera_available:
            return None
        async with self._snapshot_lock:
            now = time.monotonic()
            if self._snapshot is not None and (now - self._snapshot_ts) < self._snapshot_min_interval:
                return self._snapshot
            try:
                resp = await self._client.get("/api/v1/cameras/snap")
                if resp.status_code == 200 and resp.content:
                    self._snapshot = resp.content
                    self._snapshot_ts = now
            except httpx.RequestError:
                pass
            return self._snapshot

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    async def _resolve_storage(self) -> None:
        """Pick a writable storage name for uploads (defaults to 'usb')."""
        self._storage = "usb"
        if not self._client:
            return
        try:
            resp = await self._client.get("/api/v1/storage")
            resp.raise_for_status()
            for item in resp.json().get("storage_list", []):
                if item.get("available") and not item.get("read_only", False):
                    name = item.get("name") or item.get("path", "").strip("/")
                    if name:
                        self._storage = name
                        return
        except (httpx.RequestError, httpx.HTTPStatusError, ValueError):
            pass

    async def _probe_camera(self) -> None:
        """Detect whether the printer exposes a camera snapshot."""
        self._camera_available = False
        if not self._client:
            return
        try:
            resp = await self._client.get("/api/v1/cameras/snap")
            self._camera_available = resp.status_code == 200 and bool(resp.content)
        except httpx.RequestError:
            self._camera_available = False


async def discover_prusalink(timeout: float = 5.0) -> list[dict]:
    """Discover PrusaLink printers on the local network via mDNS."""
    try:
        from zeroconf import ServiceBrowser, Zeroconf
        import socket

        results = []
        zeroconf = Zeroconf()

        class Listener:
            def add_service(self, zc, type_, name):
                info = zc.get_service_info(type_, name)
                if info:
                    addr = socket.inet_ntoa(info.addresses[0]) if info.addresses else None
                    results.append({
                        "name": name.replace("._prusalink._tcp.local.", ""),
                        "host": addr,
                        "port": info.port,
                    })

            def remove_service(self, *_): pass
            def update_service(self, *_): pass

        browser = ServiceBrowser(zeroconf, "_prusalink._tcp.local.", Listener())
        await asyncio.sleep(timeout)
        zeroconf.close()
        return results
    except ImportError:
        return []
