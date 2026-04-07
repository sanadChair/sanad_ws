import yaml
import os

rooms_yaml_path = "src/sanad_room_manager/config/rooms.yaml"
with open(rooms_yaml_path, "r") as f:
    rooms_data = yaml.safe_load(f)
print(rooms_data["House"])
