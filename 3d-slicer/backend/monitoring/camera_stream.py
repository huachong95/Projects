"""MJPEG camera stream grabber — fetches frames from the Prusa Camera via PrusaLink."""
import asyncio
from typing import Optional

import httpx


class CameraStream:
    def __init__(self):
        self._url: Optional[str] = None
        self._latest_frame: Optional[bytes] = None
        self._frame_queue: asyncio.Queue = asyncio.Queue(maxsize=2)
        self._task: Optional[asyncio.Task] = None
        self._client: Optional[httpx.AsyncClient] = None

    def set_url(self, url: str) -> None:
        self._url = url

    async def start(self) -> None:
        if self._task and not self._task.done():
            return
        self._task = asyncio.create_task(self._stream_loop())

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            self._task = None
        if self._client:
            await self._client.aclose()
            self._client = None

    def get_latest_frame(self) -> Optional[bytes]:
        return self._latest_frame

    async def _stream_loop(self) -> None:
        if not self._url:
            return
        self._client = httpx.AsyncClient(timeout=30.0)
        try:
            async with self._client.stream("GET", self._url) as response:
                boundary = self._get_boundary(response.headers.get("content-type", ""))
                buffer = b""
                async for chunk in response.aiter_bytes(chunk_size=8192):
                    buffer += chunk
                    frames, buffer = self._extract_frames(buffer, boundary)
                    for frame in frames:
                        self._latest_frame = frame
                        if self._frame_queue.full():
                            try:
                                self._frame_queue.get_nowait()
                            except asyncio.QueueEmpty:
                                pass
                        await self._frame_queue.put(frame)
        except (httpx.RequestError, asyncio.CancelledError):
            pass
        finally:
            if self._client:
                await self._client.aclose()
                self._client = None

    def _get_boundary(self, content_type: str) -> bytes:
        for part in content_type.split(";"):
            part = part.strip()
            if part.startswith("boundary="):
                return b"--" + part[9:].encode()
        return b"--boundarydonotcross"

    def _extract_frames(self, buffer: bytes, boundary: bytes) -> tuple[list[bytes], bytes]:
        frames = []
        while boundary in buffer:
            start = buffer.find(boundary)
            end = buffer.find(boundary, start + len(boundary))
            if end == -1:
                break
            segment = buffer[start + len(boundary):end]
            header_end = segment.find(b"\r\n\r\n")
            if header_end != -1:
                jpeg = segment[header_end + 4:].strip()
                if jpeg.startswith(b"\xff\xd8"):
                    frames.append(jpeg)
            buffer = buffer[end:]
        return frames, buffer


camera_stream = CameraStream()
