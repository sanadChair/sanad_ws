import json
import re

import yaml


def _key(value):
    return re.sub(r"[^a-z0-9]+", "", str(value or "").casefold())


def load_home_context(rooms_yaml_path):
    rooms = []
    devices = []

    if rooms_yaml_path:
        with open(rooms_yaml_path, "r", encoding="utf-8") as yaml_file:
            rooms_data = yaml.safe_load(yaml_file) or {}
    else:
        rooms_data = {}

    if isinstance(rooms_data.get("House"), dict):
        rooms_data = rooms_data["House"]

    if not isinstance(rooms_data, dict):
        rooms_data = {}

    for room_key, room_info in rooms_data.items():
        if not isinstance(room_info, dict):
            continue

        room_name = str(room_info.get("name") or room_key)
        room_id = str(room_info.get("id") or "")
        pose = (room_info.get("coordinates") or {}).get("pose") or {}
        room = {
            "key": str(room_key),
            "id": room_id,
            "name": room_name,
            "pose": {
                "x": float(pose.get("x", 0.0)),
                "y": float(pose.get("y", 0.0)),
                "theta": float(pose.get("theta", 0.0)),
            },
        }
        rooms.append(room)

        for device_info in room_info.get("devices") or []:
            if not isinstance(device_info, dict):
                continue
            device = {
                "id": str(device_info.get("id") or ""),
                "name": str(device_info.get("name") or ""),
                "control_type": str(device_info.get("control_type") or ""),
                "state": int(device_info.get("state") or 0),
                "connected": bool(device_info.get("connected", False)),
                "room_key": room["key"],
                "room_id": room_id,
                "room_name": room_name,
            }
            if device["id"] or device["name"]:
                devices.append(device)

    rooms_context = [
        {"name": room["name"], "id": room["id"] or room["key"]}
        for room in rooms
    ]
    devices_context = [
        {
            "name": device["name"],
            "id": device["id"],
            "room": device["room_name"],
            "state": device["state"],
            "control_type": device["control_type"],
            "connected": device["connected"],
        }
        for device in devices
    ]

    return {
        "rooms": rooms,
        "devices": devices,
        "rooms_prompt": json.dumps(rooms_context, ensure_ascii=False),
        "devices_prompt": json.dumps(devices_context, ensure_ascii=False),
    }


def resolve_device(devices, target, room=""):
    target_key = _key(target)
    room_key = _key(room)
    if not target_key:
        return None, "missing device target"

    search_devices = devices
    if room_key:
        room_devices = [
            device
            for device in devices
            if room_key
            in {_key(device["room_name"]), _key(device["room_id"]), _key(device["room_key"])}
        ]
        if room_devices:
            search_devices = room_devices

    matches = [
        device for device in search_devices if target_key and _key(device["id"]) == target_key
    ]
    if not matches:
        matches = [
            device
            for device in search_devices
            if target_key and _key(device["name"]) == target_key
        ]

    if not matches:
        return None, f"device '{target}' was not found"
    if len(matches) > 1:
        names = ", ".join(
            f"{device['name']} in {device['room_name']}" for device in matches[:4]
        )
        return None, f"device '{target}' is ambiguous: {names}"
    return matches[0], None
