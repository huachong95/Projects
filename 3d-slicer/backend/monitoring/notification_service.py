"""Push notification service — Telegram bot, Discord webhook, generic HTTP webhook."""
import asyncio
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import httpx

logger = logging.getLogger(__name__)

EVENTS = {
    "print_started", "print_complete", "print_failed",
    "print_cancelled", "ai_alert", "filament_low",
}


@dataclass
class NotificationChannel:
    channel_id: str
    type: str           # telegram | discord | webhook
    enabled: bool = True
    events: list[str] = field(default_factory=lambda: list(EVENTS))
    # Telegram
    bot_token: str = ""
    chat_id: str = ""
    # Discord / generic webhook
    webhook_url: str = ""


class NotificationService:
    def __init__(self, data_dir: Path):
        self._path = data_dir / "notifications.json"
        self._channels: list[NotificationChannel] = self._load()

    def _load(self) -> list[NotificationChannel]:
        if not self._path.exists():
            return []
        try:
            return [NotificationChannel(**c) for c in json.loads(self._path.read_text())]
        except Exception:
            return []

    def _save(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        from dataclasses import asdict
        self._path.write_text(json.dumps([asdict(c) for c in self._channels], indent=2))

    def list_channels(self) -> list[NotificationChannel]:
        return self._channels

    def get_channel(self, channel_id: str) -> Optional[NotificationChannel]:
        return next((c for c in self._channels if c.channel_id == channel_id), None)

    def save_channel(self, channel: NotificationChannel) -> NotificationChannel:
        existing = self.get_channel(channel.channel_id)
        if existing:
            self._channels = [c if c.channel_id != channel.channel_id else channel
                              for c in self._channels]
        else:
            self._channels.append(channel)
        self._save()
        return channel

    def delete_channel(self, channel_id: str) -> bool:
        before = len(self._channels)
        self._channels = [c for c in self._channels if c.channel_id != channel_id]
        if len(self._channels) < before:
            self._save()
            return True
        return False

    async def notify(
        self,
        event: str,
        message: str,
        snapshot: Optional[bytes] = None,
    ) -> None:
        if event not in EVENTS:
            return
        tasks = [
            self._dispatch(ch, event, message, snapshot)
            for ch in self._channels
            if ch.enabled and event in ch.events
        ]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _dispatch(
        self,
        channel: NotificationChannel,
        event: str,
        message: str,
        snapshot: Optional[bytes],
    ) -> None:
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                if channel.type == "telegram":
                    await self._telegram(client, channel, message, snapshot)
                elif channel.type == "discord":
                    await self._discord(client, channel, message, snapshot)
                elif channel.type == "webhook":
                    await self._webhook(client, channel, event, message)
        except Exception as exc:
            logger.warning("Notification failed for channel %s: %s", channel.channel_id, exc)

    async def _telegram(
        self,
        client: httpx.AsyncClient,
        ch: NotificationChannel,
        message: str,
        snapshot: Optional[bytes],
    ) -> None:
        base = f"https://api.telegram.org/bot{ch.bot_token}"
        if snapshot:
            await client.post(
                f"{base}/sendPhoto",
                data={"chat_id": ch.chat_id, "caption": message},
                files={"photo": ("snap.jpg", snapshot, "image/jpeg")},
            )
        else:
            await client.post(
                f"{base}/sendMessage",
                json={"chat_id": ch.chat_id, "text": message},
            )

    async def _discord(
        self,
        client: httpx.AsyncClient,
        ch: NotificationChannel,
        message: str,
        snapshot: Optional[bytes],
    ) -> None:
        if snapshot:
            await client.post(
                ch.webhook_url,
                data={"payload_json": json.dumps({"content": message})},
                files={"file": ("snap.jpg", snapshot, "image/jpeg")},
            )
        else:
            await client.post(ch.webhook_url, json={"content": message})

    async def _webhook(
        self,
        client: httpx.AsyncClient,
        ch: NotificationChannel,
        event: str,
        message: str,
    ) -> None:
        await client.post(
            ch.webhook_url,
            json={"event": event, "message": message},
        )

    async def test_channel(self, channel_id: str) -> bool:
        ch = self.get_channel(channel_id)
        if not ch:
            return False
        try:
            await self.notify.__wrapped__ if hasattr(self.notify, '__wrapped__') else None
            async with httpx.AsyncClient(timeout=10) as client:
                await self._dispatch(ch, "print_complete",
                                     "🎉 Test notification from 3D Slicer app", None)
            return True
        except Exception:
            return False
