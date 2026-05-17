import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from monitoring.notification_service import NotificationChannel, NotificationService, EVENTS

router = APIRouter()
_notif: Optional[NotificationService] = None


def init(notification_service: NotificationService) -> None:
    global _notif
    _notif = notification_service


class ChannelCreate(BaseModel):
    type: str                    # telegram | discord | webhook
    enabled: bool = True
    events: list[str] = list(EVENTS)
    bot_token: str = ""
    chat_id: str = ""
    webhook_url: str = ""


@router.get("")
async def list_channels():
    channels = _notif.list_channels()
    # Redact tokens in list response
    result = []
    for c in channels:
        d = {
            "channel_id": c.channel_id,
            "type": c.type,
            "enabled": c.enabled,
            "events": c.events,
            "webhook_url": c.webhook_url[:40] + "…" if len(c.webhook_url) > 40 else c.webhook_url,
            "chat_id": c.chat_id,
            "has_token": bool(c.bot_token),
        }
        result.append(d)
    return {"channels": result}


@router.post("", status_code=201)
async def create_channel(req: ChannelCreate):
    invalid = [e for e in req.events if e not in EVENTS]
    if invalid:
        raise HTTPException(400, f"Unknown events: {invalid}. Valid: {sorted(EVENTS)}")
    if req.type not in ("telegram", "discord", "webhook"):
        raise HTTPException(400, "type must be telegram, discord, or webhook")
    channel = NotificationChannel(
        channel_id=str(uuid.uuid4()),
        **req.model_dump(),
    )
    _notif.save_channel(channel)
    return {"channel_id": channel.channel_id, "status": "created"}


@router.patch("/{channel_id}")
async def update_channel(channel_id: str, req: ChannelCreate):
    existing = _notif.get_channel(channel_id)
    if not existing:
        raise HTTPException(404, "Channel not found")
    updated = NotificationChannel(channel_id=channel_id, **req.model_dump())
    _notif.save_channel(updated)
    return {"status": "updated"}


@router.delete("/{channel_id}")
async def delete_channel(channel_id: str):
    if not _notif.delete_channel(channel_id):
        raise HTTPException(404, "Channel not found")
    return {"status": "deleted"}


@router.post("/{channel_id}/test")
async def test_channel(channel_id: str):
    ch = _notif.get_channel(channel_id)
    if not ch:
        raise HTTPException(404, "Channel not found")
    from monitoring.camera_stream import camera_stream
    snapshot = camera_stream.get_latest_frame()
    await _notif.notify("print_complete", "🎉 Test from your 3D Slicer app!", snapshot)
    return {"status": "sent"}


@router.get("/events")
async def list_events():
    return {"events": sorted(EVENTS)}
