#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
import threading
import os
import yaml
import time


def load_dotenv():
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('sanad_bringup')
        env_path = os.path.join(share_dir, '.env')
        if os.path.exists(env_path):
            with open(env_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    if '=' in line:
                        k, v = line.split('=', 1)
                        os.environ[k.strip()] = v.strip().strip('"').strip("'")
    except Exception:
        pass

class TelegramManagerNode(Node):
    def __init__(self):
        super().__init__('telegram_node')
        
        # Subscriptions
        self.send_sub = self.create_subscription(
            String,
            '/robot/telegram/send',
            self.send_callback,
            10
        )
        self.clear_sub = self.create_subscription(
            String,
            '/robot/telegram/clear',
            self.clear_callback,
            10
        )
        
        # Paths for config files
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sanad_voice_control')
            self.telegram_contacts_path = os.path.join(share_dir, 'config', 'telegram_contacts.yaml')
            self.telegram_config_path = os.path.join(share_dir, 'config', 'telegram_config.yaml')
            self.telegram_messages_path = os.path.join(share_dir, 'config', 'telegram_messages.yaml')
        except Exception:
            self.telegram_contacts_path = 'config/telegram_contacts.yaml'
            self.telegram_config_path = 'config/telegram_config.yaml'
            self.telegram_messages_path = 'config/telegram_messages.yaml'
        
        # Ensure files exist
        os.makedirs(os.path.dirname(self.telegram_messages_path), exist_ok=True)
        if not os.path.exists(self.telegram_messages_path):
            with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                yaml.dump([], f)
                
        # Load Telegram Contacts
        try:
            with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                self.telegram_contacts = yaml.safe_load(file) or {}
        except FileNotFoundError:
            self.get_logger().warning("telegram_contacts.yaml not found! Contacts won't load.")
            self.telegram_contacts = {}

        # Load Telegram Token (prioritize environment/.env)
        load_dotenv()
        self.telegram_token = os.environ.get("TELEGRAM_BOT_TOKEN", None)
        
        if not self.telegram_token:
            try:
                with open(self.telegram_config_path, 'r', encoding='utf-8') as file:
                    t_config = yaml.safe_load(file) or {}
                    self.telegram_token = t_config.get('telegram_token', None)
            except FileNotFoundError:
                self.get_logger().warning("telegram_config.yaml not found! Telegram won't work.")
                self.telegram_token = None
            
        # --- START TELEGRAM LISTENER THREAD ---
        self.telegram_offset = 0
        if self.telegram_token:
            self.get_logger().info("📡 Starting Telegram polling thread (Inbox Mode)...")
            self.telegram_thread = threading.Thread(target=self.telegram_polling_loop, daemon=True)
            self.telegram_thread.start()
        else:
            self.get_logger().error("Telegram token missing. Polling thread not started.")
            
        self.get_logger().info("✅ Telegram Node Ready!")

    def telegram_polling_loop(self):
        """Checks for new messages and SAVES them to a YAML file."""
        allowed_chat_ids = [str(v) for v in self.telegram_contacts.values()]
        
        while rclpy.ok():
            url = f"https://api.telegram.org/bot{self.telegram_token}/getUpdates"
            params = {"offset": self.telegram_offset, "timeout": 30}
            
            try:
                response = requests.get(url, params=params, timeout=40)
                if response.status_code == 200:
                    data = response.json()
                    for update in data.get("result", []):
                        self.telegram_offset = update["update_id"] + 1
                        
                        message = update.get("message", {})
                        text = message.get("text", "")
                        chat_id = str(message.get("chat", {}).get("id", ""))
                        
                        if text:
                            if chat_id in allowed_chat_ids:
                                # Reverse lookup to find sender's name
                                sender_name = "Unknown"
                                for name, cid in self.telegram_contacts.items():
                                    if str(cid) == chat_id:
                                        sender_name = name
                                        break
                                        
                                self.get_logger().info(f"📩 [New Inbox Message from {sender_name}]: {text}")
                                
                                # Save the message to our unread YAML file
                                try:
                                    unread_msgs = []
                                    if os.path.exists(self.telegram_messages_path):
                                        with open(self.telegram_messages_path, 'r', encoding='utf-8') as f:
                                            unread_msgs = yaml.safe_load(f) or []
                                            
                                    if not isinstance(unread_msgs, list):
                                        unread_msgs = []

                                    # Migrate existing old format messages on-the-fly
                                    user_map = {}
                                    for entry in unread_msgs:
                                        if not isinstance(entry, dict):
                                            continue
                                        if "sender" in entry and "message" in entry:
                                            sender = entry["sender"]
                                            msg_text = entry["message"]
                                            if sender not in user_map:
                                                user_map[sender] = []
                                            user_map[sender].append({"message": msg_text})
                                        elif "user" in entry and "messages" in entry:
                                            user = entry["user"]
                                            msgs = entry["messages"]
                                            if not isinstance(msgs, list):
                                                msgs = []
                                            if user not in user_map:
                                                user_map[user] = []
                                            user_map[user].extend(msgs)
                                    
                                    # Group/add new message
                                    if sender_name not in user_map:
                                        user_map[sender_name] = []
                                    user_map[sender_name].append({"message": text})

                                    # Rebuild unread list grouped by user
                                    unread_msgs = [{"user": u, "messages": m} for u, m in user_map.items()]
                                    
                                    with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                                        yaml.dump(unread_msgs, f, allow_unicode=True)
                                except Exception as e:
                                    self.get_logger().error(f"Error saving telegram message: {e}")
                            else:
                                self.get_logger().warning(f"⚠️ [Unauthorized Telegram user]: ChatID {chat_id} said: '{text}'")
            except requests.exceptions.Timeout:
                pass 
            except Exception as e:
                self.get_logger().error(f"Telegram polling error: {e}")
                time.sleep(5)

    def send_telegram_message(self, chat_id, text):
        if not self.telegram_token:
            self.get_logger().error("Telegram Bot Token is missing from config!")
            return False
            
        url = f"https://api.telegram.org/bot{self.telegram_token}/sendMessage"
        payload = {"chat_id": chat_id, "text": text}
        try:
            response = requests.post(url, json=payload)
            return response.status_code == 200
        except Exception as e:
            self.get_logger().error(f"Failed to send telegram message: {e}")
            return False

    def send_callback(self, msg):
        """Processes sending telegram messages requested via the send topic."""
        try:
            data = json.loads(msg.data)
            contact_name = data.get('contact', '').lower()
            message_text = data.get('message', '')
            
            # Reload contacts config in case it changed dynamically
            try:
                with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                    self.telegram_contacts = yaml.safe_load(file) or {}
            except Exception:
                pass

            if contact_name in self.telegram_contacts:
                target_chat_id = self.telegram_contacts[contact_name]
                success = self.send_telegram_message(target_chat_id, message_text)
                if success:
                    self.get_logger().info(f"📱 TELEGRAM: Sent to {contact_name} -> {message_text}")
                else:
                    self.get_logger().error("❌ TELEGRAM Error: API request failed.")
            else:
                self.get_logger().error(f"❌ TELEGRAM Error: Contact '{contact_name}' not found in contacts YAML.")
        except Exception as e:
            self.get_logger().error(f"Error parsing telegram send payload: {e}")

    def clear_callback(self, msg):
        """Clears the inbox YAML file based on scope."""
        try:
            data = None
            try:
                data = json.loads(msg.data)
            except Exception:
                # If not JSON, treat as raw command
                pass

            if data is not None and isinstance(data, dict):
                scope = data.get("scope", "all")
                if scope == "sender":
                    sender_to_clear = data.get("sender", "").strip().lower()
                    if sender_to_clear:
                        # Filter out messages from this sender
                        unread_msgs = []
                        if os.path.exists(self.telegram_messages_path):
                            with open(self.telegram_messages_path, 'r', encoding='utf-8') as f:
                                unread_msgs = yaml.safe_load(f) or []
                        
                        if not isinstance(unread_msgs, list):
                            unread_msgs = []

                        # Filter using the new "user" key, but also support "sender" key migration/clearing if present
                        filtered_msgs = []
                        for m in unread_msgs:
                            if not isinstance(m, dict):
                                continue
                            user_name = m.get("user") or m.get("sender", "")
                            if user_name.strip().lower() != sender_to_clear:
                                # Ensure it gets stored as the new format
                                if "sender" in m:
                                    filtered_msgs.append({
                                        "user": m["sender"],
                                        "messages": [{"message": m["message"]}]
                                    })
                                else:
                                    filtered_msgs.append(m)
                        
                        with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                            yaml.dump(filtered_msgs, f, allow_unicode=True)
                        self.get_logger().info(f"📭 Inbox messages from '{sender_to_clear}' marked as read and cleared.")
                        return
                
            # Fallback: Clear everything
            with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                yaml.dump([], f)
            self.get_logger().info("📭 Entire inbox marked as read and cleared.")
        except Exception as e:
            self.get_logger().error(f"❌ Error clearing inbox: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelegramManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
