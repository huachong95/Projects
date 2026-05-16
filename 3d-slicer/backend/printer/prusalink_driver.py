"""PrusaLink REST API driver for Prusa Mk4 (and other PrusaLink-enabled printers)."""
import asyncio
from enum import Enum
from typing import Optional

import httpx
from pydantic import BaseModel


class PrinterState(str, Enum):
    IDLE = "IDLE"
    PRINTING = "PRINTING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"
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
    camera_url: Optional[str] = None


class PrusaLinkDriver:
    def __init__(self, host: str, api_key: str, timeout: float = 10.0):
        self.base_url = f"http://{host}"
        self.headers = {"X-Api-Key": api_key}
        self.timeout = timeout
        self._client: Optional[httpx.AsyncClient] = None
        self._camera_url = f"http://{host}/camera.mjpeg"

    async def connect(self) -> bool:
        self._client = httpx.AsyncClient(
            base_url=self.base_url,
            headers=self.headers,
            timeout=self.timeout,
        )
        try:
            resp = await self._client.get("/api/v1/info")
            return resp.status_code == 200
        except httpx.RequestError:
            return False

    async def disconnect(self) -> None:
        if self._client:
            await self._client.aclose()
            self._client = None

    async def get_status(self) -> PrinterStatus:
        if not self._client:
            return PrinterStatus()
        try:
            resp = await self._client.get("/api/v1/status")
            resp.raise_for_status()
            data = resp.json()

            printer = data.get("printer", {})
            job = data.get("job", {})

            state_str = printer.get("state", "UNKNOWN").upper()
            try:
                state = PrinterState(state_str)
            except ValueError:
                state = PrinterState.UNKNOWN

            return PrinterStatus(
                state=state,
                temp_hotend=printer.get("temp_nozzle", 0.0),
                temp_hotend_target=printer.get("target_nozzle", 0.0),
                temp_bed=printer.get("temp_bed", 0.0),
                temp_bed_target=printer.get("target_bed", 0.0),
                progress_percent=job.get("progress", 0.0),
                filename=job.get("file", {}).get("name"),
                eta_seconds=job.get("time_remaining"),
                camera_url=self._camera_url,
            )
        except (httpx.RequestError, httpx.HTTPStatusError, KeyError):
            return PrinterStatus()

    async def upload_gcode(self, gcode_path: str, filename: str) -> bool:
        if not self._client:
            return False
        try:
            with open(gcode_path, "rb") as f:
                resp = await self._client.put(
                    f"/api/v1/files/usb/{filename}",
                    content=f.read(),
                    headers={**self.headers, "Content-Type": "text/x.gcode"},
                )
            return resp.status_code in (200, 201)
        except (httpx.RequestError, OSError):
            return False

    async def start_print(self, filename: str) -> bool:
        if not self._client:
            return False
        try:
            resp = await self._client.post(
                f"/api/v1/files/usb/{filename}",
                json={"command": "print"},
            )
            return resp.status_code == 204
        except httpx.RequestError:
            return False

    async def pause(self) -> bool:
        return await self._send_job_command("pause")

    async def resume(self) -> bool:
        return await self._send_job_command("resume")

    async def cancel(self) -> bool:
        return await self._send_job_command("cancel")

    async def send_gcode(self, command: str) -> bool:
        if not self._client:
            return False
        try:
            resp = await self._client.post(
                "/api/v1/printer/gcode",
                json={"script": command},
            )
            return resp.status_code == 204
        except httpx.RequestError:
            return False

    def get_camera_url(self) -> str:
        return self._camera_url

    async def _send_job_command(self, command: str) -> bool:
        if not self._client:
            return False
        try:
            resp = await self._client.put(
                "/api/v1/job",
                json={"command": command},
            )
            return resp.status_code == 204
        except httpx.RequestError:
            return False


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
