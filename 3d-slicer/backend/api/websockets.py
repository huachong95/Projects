import asyncio
import json
from datetime import datetime, timezone
from typing import Optional

from fastapi import WebSocket, WebSocketDisconnect


class WebSocketManager:
    def __init__(self):
        self._connections: dict[str, list[WebSocket]] = {}

    async def connect(self, channel: str, websocket: WebSocket) -> None:
        await websocket.accept()
        self._connections.setdefault(channel, []).append(websocket)

    def disconnect(self, channel: str, websocket: WebSocket) -> None:
        channel_conns = self._connections.get(channel, [])
        if websocket in channel_conns:
            channel_conns.remove(websocket)

    async def broadcast(self, channel: str, msg_type: str, data: dict) -> None:
        message = json.dumps({
            "type": msg_type,
            "data": data,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        })
        dead: list[WebSocket] = []
        for ws in self._connections.get(channel, []):
            try:
                await ws.send_text(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(channel, ws)

    async def send_to(self, websocket: WebSocket, msg_type: str, data: dict) -> None:
        message = json.dumps({
            "type": msg_type,
            "data": data,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        })
        await websocket.send_text(message)


ws_manager = WebSocketManager()
