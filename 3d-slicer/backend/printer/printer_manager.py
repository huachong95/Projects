"""Routes connections to the appropriate printer driver."""
from enum import Enum
from typing import Optional, Union

from pydantic import BaseModel

from printer.prusalink_driver import PrusaLinkDriver, PrinterStatus, PrinterState


class ConnectionType(str, Enum):
    PRUSALINK = "prusalink"
    PRUSA_CONNECT = "prusa_connect"
    OCTOPRINT = "octoprint"
    SERIAL = "serial"


class ConnectionConfig(BaseModel):
    type: ConnectionType
    host: Optional[str] = None
    api_key: Optional[str] = None
    port: Optional[str] = None
    baud_rate: int = 115200


class PrinterManager:
    def __init__(self):
        self._driver: Optional[PrusaLinkDriver] = None
        self._config: Optional[ConnectionConfig] = None
        # Metadata about a print we just kicked off, so the status poll loop
        # can attribute slice job + filament usage to the history record.
        self._pending_print: Optional[dict] = None

    def set_pending_print(self, filename: str, slice_job_id: Optional[str],
                          filament_used_g: float) -> None:
        self._pending_print = {
            "filename": filename,
            "slice_job_id": slice_job_id,
            "filament_used_g": filament_used_g,
        }

    def take_pending_print(self) -> Optional[dict]:
        pending = self._pending_print
        self._pending_print = None
        return pending

    async def connect(self, config: ConnectionConfig) -> bool:
        await self.disconnect()
        self._config = config

        if config.type == ConnectionType.PRUSALINK:
            if not config.host or not config.api_key:
                return False
            self._driver = PrusaLinkDriver(config.host, config.api_key)
            return await self._driver.connect()

        # Other drivers (OctoPrint, serial) can be added here following the same pattern
        return False

    async def disconnect(self) -> None:
        if self._driver:
            await self._driver.disconnect()
            self._driver = None
        self._config = None

    @property
    def is_connected(self) -> bool:
        return self._driver is not None

    async def get_status(self) -> PrinterStatus:
        if not self._driver:
            return PrinterStatus()
        return await self._driver.get_status()

    async def upload_and_print(self, gcode_path: str, filename: str) -> bool:
        if not self._driver:
            return False
        # Upload and start in one step via the Print-After-Upload header; this
        # avoids a race where the file isn't registered yet when we POST print.
        return await self._driver.upload_gcode(gcode_path, filename, print_after=True)

    async def pause(self) -> bool:
        return await self._driver.pause() if self._driver else False

    async def resume(self) -> bool:
        return await self._driver.resume() if self._driver else False

    async def cancel(self) -> bool:
        return await self._driver.cancel() if self._driver else False

    async def send_gcode(self, command: str) -> bool:
        return await self._driver.send_gcode(command) if self._driver else False

    async def get_snapshot(self) -> Optional[bytes]:
        if isinstance(self._driver, PrusaLinkDriver):
            return await self._driver.get_snapshot()
        return None

    @property
    def camera_available(self) -> bool:
        return isinstance(self._driver, PrusaLinkDriver) and self._driver.camera_available

    def supports_gcode(self) -> bool:
        return self._driver is not None and self._driver.supports_gcode()
