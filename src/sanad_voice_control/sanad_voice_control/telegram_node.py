#!/usr/bin/env python3
import json
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from sanad_interfaces.srv import (
    GetTelegramGroups,
    GetTelegramMessages,
    GetTelegramUsers,
    SendTelegramMessage,
)


GET_TELEGRAM_GROUPS_SERVICE = "/telegram/chat/groups"
GET_TELEGRAM_USERS_SERVICE = "/telegram/chat/users"
GET_TELEGRAM_MESSAGES_SERVICE = "/telegram/chat/messages"
SEND_TELEGRAM_MESSAGE_SERVICE = "/telegram/chat/send"


class TelegramManagerNode(Node):
    def __init__(self):
        super().__init__("telegram_node")

        self.declare_parameter("telegram_id", 0)
        self.declare_parameter("telegram_email", "")
        self.declare_parameter("poll_interval_sec", 30.0)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("dialogs_limit", 50)
        self.declare_parameter("messages_limit", 20)

        self.telegram_id = int(self.get_parameter("telegram_id").value)
        self.telegram_email = self.get_parameter("telegram_email").value
        self.poll_interval_sec = float(self.get_parameter("poll_interval_sec").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.dialogs_limit = int(self.get_parameter("dialogs_limit").value)
        self.messages_limit = int(self.get_parameter("messages_limit").value)

        self.get_groups_client = self.create_client(
            GetTelegramGroups,
            GET_TELEGRAM_GROUPS_SERVICE,
        )
        self.get_users_client = self.create_client(
            GetTelegramUsers,
            GET_TELEGRAM_USERS_SERVICE,
        )
        self.get_messages_client = self.create_client(
            GetTelegramMessages,
            GET_TELEGRAM_MESSAGES_SERVICE,
        )
        self.send_message_client = self.create_client(
            SendTelegramMessage,
            SEND_TELEGRAM_MESSAGE_SERVICE,
        )

        self.send_sub = self.create_subscription(
            String,
            "/robot/telegram/send",
            self.send_callback,
            10,
        )
        self.clear_sub = self.create_subscription(
            String,
            "/robot/telegram/clear",
            self.clear_callback,
            10,
        )

        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory("sanad_voice_control")
            self.telegram_contacts_path = os.path.join(
                share_dir, "config", "telegram_contacts.yaml"
            )
            self.telegram_messages_path = os.path.join(
                share_dir, "config", "telegram_messages.yaml"
            )
        except Exception:
            self.telegram_contacts_path = "config/telegram_contacts.yaml"
            self.telegram_messages_path = "config/telegram_messages.yaml"

        os.makedirs(os.path.dirname(self.telegram_messages_path), exist_ok=True)
        if not os.path.exists(self.telegram_messages_path):
            self._write_yaml(self.telegram_messages_path, [])
        if not os.path.exists(self.telegram_contacts_path):
            self._write_yaml(self.telegram_contacts_path, {})

        self.file_lock = threading.Lock()
        self.service_lock = threading.Lock()
        self.seen_message_ids = {}
        self.dialog_names = {}
        self.reported_missing_services = set()
        self.reported_empty_dialogs = False
        self.stop_event = threading.Event()

        self.telegram_thread = threading.Thread(
            target=self.telegram_polling_loop,
            daemon=True,
        )
        self.telegram_thread.start()

        self.get_logger().info(
            "Telegram node ready. Using client services: "
            f"groups={GET_TELEGRAM_GROUPS_SERVICE}, users={GET_TELEGRAM_USERS_SERVICE}, "
            f"messages={GET_TELEGRAM_MESSAGES_SERVICE}, send={SEND_TELEGRAM_MESSAGE_SERVICE}"
        )
        self.get_logger().info(
            f"Telegram polling thread started. poll_interval={self.poll_interval_sec:.1f}s, "
            f"service_timeout={self.service_timeout_sec:.1f}s"
        )

    def _read_yaml(self, path, fallback):
        try:
            with open(path, "r", encoding="utf-8") as file:
                return yaml.safe_load(file) or fallback
        except Exception:
            return fallback

    def _write_yaml(self, path, data):
        with open(path, "w", encoding="utf-8") as file:
            yaml.dump(data, file, allow_unicode=True)

    def _call_service(self, client, request, service_name, timeout_sec=10.0):
        with self.service_lock:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                if service_name not in self.reported_missing_services:
                    self.reported_missing_services.add(service_name)
                    self.get_logger().warning(
                        f"Telegram: service '{service_name}' is not available after {timeout_sec:.1f}s."
                    )
                return None

            done = threading.Event()
            result = {"response": None, "error": None}
            future = client.call_async(request)

            def on_done(fut):
                try:
                    result["response"] = fut.result()
                except Exception as exc:
                    result["error"] = exc
                finally:
                    done.set()

            future.add_done_callback(on_done)
            if not done.wait(timeout_sec):
                self.get_logger().warning(
                    f"Telegram: service '{service_name}' did not respond within {timeout_sec:.1f}s."
                )
                return None
            if result["error"]:
                raise result["error"]
            return result["response"]

    def _dialog_name(self, dialog):
        return (dialog.name or dialog.title or dialog.id).strip()

    def _load_contacts(self):
        contacts = self._read_yaml(self.telegram_contacts_path, {})
        return contacts if isinstance(contacts, dict) else {}

    def _save_messages(self, sender_name, messages):
        if not messages:
            return

        with self.file_lock:
            unread_msgs = self._read_yaml(self.telegram_messages_path, [])
            if not isinstance(unread_msgs, list):
                unread_msgs = []

            grouped = {}
            for entry in unread_msgs:
                if not isinstance(entry, dict):
                    continue
                if "sender" in entry and "message" in entry:
                    user = entry["sender"]
                    grouped.setdefault(user, []).append({"message": entry["message"]})
                elif "user" in entry and "messages" in entry:
                    user = entry["user"]
                    existing = entry["messages"] if isinstance(entry["messages"], list) else []
                    grouped.setdefault(user, []).extend(existing)

            grouped.setdefault(sender_name, [])
            for message in messages:
                grouped[sender_name].append({"message": message.message})

            self._write_yaml(
                self.telegram_messages_path,
                [{"user": user, "messages": items} for user, items in grouped.items()],
            )

    def _sync_dialogs(self):
        options = json.dumps(
            {
                "limit": self.dialogs_limit,
                "offsetDate": 0,
                "offsetId": 0,
                "ignorePinned": False,
                "ignoreMigrated": False,
                "folder": 0,
                "archived": False,
            }
        )

        groups_req = GetTelegramGroups.Request()
        groups_req.id = self.telegram_id
        groups_req.email = self.telegram_email
        groups_req.options = options

        users_req = GetTelegramUsers.Request()
        users_req.id = self.telegram_id
        users_req.email = self.telegram_email
        users_req.options = options

        groups_res = self._call_service(
            self.get_groups_client,
            groups_req,
            GET_TELEGRAM_GROUPS_SERVICE,
            self.service_timeout_sec,
        )
        users_res = self._call_service(
            self.get_users_client,
            users_req,
            GET_TELEGRAM_USERS_SERVICE,
            self.service_timeout_sec,
        )
        dialogs = []
        if groups_res:
            dialogs.extend(groups_res.groups)
        if users_res:
            dialogs.extend(users_res.users)

        if not dialogs and not self.reported_empty_dialogs:
            self.reported_empty_dialogs = True
            self.get_logger().warning(
                "Telegram client returned no dialogs. Check telegram_id/email and the client session."
            )

        contacts = self._load_contacts()
        updated_contacts = dict(contacts)
        for dialog in dialogs:
            name = self._dialog_name(dialog)
            if not name:
                continue
            self.dialog_names[dialog.id] = name
            updated_contacts.setdefault(name.lower(), dialog.id)

        if updated_contacts != contacts:
            with self.file_lock:
                self._write_yaml(self.telegram_contacts_path, updated_contacts)

        return dialogs

    def _fetch_messages(self, dialog):
        req = GetTelegramMessages.Request()
        req.id = self.telegram_id
        req.email = self.telegram_email
        req.chat = dialog.id
        req.options = json.dumps(
            {
                "limit": self.messages_limit,
                "offsetId": 0,
                "minId": 0,
                "maxId": 0,
                "addOffset": 0,
                "search": "",
                "reverse": False,
            }
        )

        res = self._call_service(
            self.get_messages_client,
            req,
            GET_TELEGRAM_MESSAGES_SERVICE,
            self.service_timeout_sec,
        )
        if not res:
            return []
        return sorted(res.messages, key=lambda item: item.id)

    def telegram_polling_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                dialogs = self._sync_dialogs()
                for dialog in dialogs:
                    if dialog.unread_count <= 0:
                        continue

                    dialog_name = self.dialog_names.get(dialog.id, self._dialog_name(dialog))
                    messages = self._fetch_messages(dialog)
                    if not messages:
                        continue

                    last_seen = self.seen_message_ids.get(dialog.id)
                    latest_id = max(message.id for message in messages)
                    if last_seen is None:
                        self.seen_message_ids[dialog.id] = latest_id
                        if dialog.unread_count <= 0:
                            continue
                        threshold = latest_id - dialog.unread_count
                        new_messages = [
                            message
                            for message in messages
                            if not message.out and message.message and message.id > threshold
                        ]
                    else:
                        new_messages = [
                            message
                            for message in messages
                            if not message.out and message.message and message.id > last_seen
                        ]
                        self.seen_message_ids[dialog.id] = latest_id

                    if new_messages:
                        self._save_messages(dialog_name, new_messages)
                        self.get_logger().info(
                            f"Telegram inbox: saved {len(new_messages)} new message(s) from {dialog_name}"
                        )
            except Exception as exc:
                self.get_logger().error(f"Telegram polling error: {exc}")

            self.stop_event.wait(self.poll_interval_sec)

    def send_callback(self, msg):
        try:
            data = json.loads(msg.data)
            contact_name = data.get("contact", "").strip().lower()
            message_text = data.get("message", "")
            options = data.get("options", {})

            contacts = self._load_contacts()
            if contact_name not in contacts:
                self.get_logger().error(
                    f"TELEGRAM: contact '{contact_name}' not found in contacts YAML."
                )
                return

            req = SendTelegramMessage.Request()
            req.id = self.telegram_id
            req.email = self.telegram_email
            req.chat = str(contacts[contact_name])
            req.message = message_text
            req.options = json.dumps(options if isinstance(options, dict) else {})

            res = self._call_service(
                self.send_message_client,
                req,
                SEND_TELEGRAM_MESSAGE_SERVICE,
                self.service_timeout_sec,
            )
            if res and res.success:
                self.get_logger().info(f"TELEGRAM: sent to {contact_name}")
            else:
                self.get_logger().error("TELEGRAM: send service returned failure.")
        except Exception as exc:
            self.get_logger().error(f"Error parsing telegram send payload: {exc}")

    def clear_callback(self, msg):
        try:
            data = None
            try:
                data = json.loads(msg.data)
            except Exception:
                pass

            if isinstance(data, dict) and data.get("scope") == "sender":
                sender_to_clear = data.get("sender", "").strip().lower()
                if sender_to_clear:
                    with self.file_lock:
                        unread_msgs = self._read_yaml(self.telegram_messages_path, [])
                        if not isinstance(unread_msgs, list):
                            unread_msgs = []

                        filtered_msgs = []
                        for item in unread_msgs:
                            if not isinstance(item, dict):
                                continue
                            user_name = item.get("user") or item.get("sender", "")
                            if user_name.strip().lower() == sender_to_clear:
                                continue
                            if "sender" in item and "message" in item:
                                filtered_msgs.append(
                                    {
                                        "user": item["sender"],
                                        "messages": [{"message": item["message"]}],
                                    }
                                )
                            else:
                                filtered_msgs.append(item)

                        self._write_yaml(self.telegram_messages_path, filtered_msgs)
                    self.get_logger().info(
                        f"Telegram inbox messages from '{sender_to_clear}' cleared."
                    )
                    return

            with self.file_lock:
                self._write_yaml(self.telegram_messages_path, [])
            self.get_logger().info("Telegram inbox cleared.")
        except Exception as exc:
            self.get_logger().error(f"Error clearing Telegram inbox: {exc}")

    def destroy_node(self):
        self.stop_event.set()
        if self.telegram_thread.is_alive():
            self.telegram_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelegramManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
