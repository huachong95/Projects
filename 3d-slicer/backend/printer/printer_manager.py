"""Routes connections to the appropriate printer driver."""
import time
from collections import deque
from enum import Enum
from typing import Optional

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


class TempReading(BaseModel):
    ts: float
    hotend: float
    hotend_target: float
    bed: float
    bed_target: float


class PrinterManager:
    _HISTORY_MAXLEN = 120  # ~2 minutes at 1 reading/sec

    def __init__(self):
        self._driver: Optional[PrusaLinkDriver] = None
        self._config: Optional[ConnectionConfig] = None
        self._temp_history: deque[TempReading] = deque(maxlen=self._HISTORY_MAXLEN)

    async def connect(self, config: ConnectionConfig) -> bool:
        await self.disconnect()
        self._config = config
        if config.type == ConnectionType.PRUSALINK:
            if not config.host or not config.api_key:
                return False
            self._driver = PrusaLinkDriver(config.host, config.api_key)
            return await self._driver.connect()
        return False

    async def disconnect(self) -> None:
        if self._driver:
            await self._driver.disconnect()
            self._driver = None
        self._config = None
        self._temp_history.clear()

    @property
    def is_connected(self) -> bool:
        return self._driver is not None

    async def get_status(self) -> PrinterStatus:
        if not self._driver:
            return PrinterStatus()
        status = await self._driver.get_status()
        self._temp_history.append(TempReading(
            ts=time.time(),
            hotend=status.temp_hotend,
            hotend_target=status.temp_hotend_target,
            bed=status.temp_bed,
            bed_target=status.temp_bed_target,
        ))
        return status

    def get_temp_history(self) -> list[TempReading]:
        return list(self._temp_history)

    async def upload_and_print(self, gcode_path: str, filename: str) -> bool:
        if not self._driver:
            return False
        if not await self._driver.upload_gcode(gcode_path, filename):
            return False
        return await self._driver.start_print(filename)

    async def pause(self) -> bool:
        return await self._driver.pause() if self._driver else False

    async def resume(self) -> bool:
        return await self._driver.resume() if self._driver else False

    async def cancel(self) -> bool:
        return await self._driver.cancel() if self._driver else False

    async def send_gcode(self, command: str) -> bool:
        return await self._driver.send_gcode(command) if self._driver else False

    async def move_axis(self, axis: str, distance: float, speed: int = 3000) -> bool:
        """Relative move on a single axis."""
        cmds = [
            "G91",  # relative positioning
            f"G1 {axis.upper()}{distance:+.2f} F{speed}",
            "G90",  # back to absolute
        ]
        for cmd in cmds:
            if not await self.send_gcode(cmd):
                return False
        return True

    async def home(self, axes: list[str] | None = None) -> bool:
        """Home specified axes, or all axes if none given."""
        if axes:
            cmd = "G28 " + " ".join(a.upper() for a in axes)
        else:
            cmd = "G28"
        return await self.send_gcode(cmd)

    async def set_temperature(self, hotend: float | None = None, bed: float | None = None) -> bool:
        ok = True
        if hotend is not None:
            ok = ok and await self.send_gcode(f"M104 S{hotend:.0f}")
        if bed is not None:
            ok = ok and await self.send_gcode(f"M140 S{bed:.0f}")
        return ok

    async def set_fan(self, speed_percent: int) -> bool:
        """Set fan speed 0-100%."""
        pwm = int(speed_percent / 100 * 255)
        return await self.send_gcode(f"M106 S{pwm}")

    async def extrude(self, distance_mm: float, speed_mm_per_min: int = 300) -> bool:
        """Extrude (positive) or retract (negative) filament."""
        cmds = [
            "M83",  # relative extruder
            f"G1 E{distance_mm:+.1f} F{speed_mm_per_min}",
            "M82",  # absolute extruder
        ]
        for cmd in cmds:
            if not await self.send_gcode(cmd):
                return False
        return True

    def get_camera_url(self) -> Optional[str]:
        if isinstance(self._driver, PrusaLinkDriver):
            return self._driver.get_camera_url()
        return None
