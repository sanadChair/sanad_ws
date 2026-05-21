#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
import threading
import re
import os
import yaml
import time

# --- KOBOLDCPP CONFIGURATION ---
KOBOLD_URL = "http://localhost:5001/v1/chat/completions"

class LLMCommander(Node):
    def __init__(self):
        super().__init__('llm_commander')
        
        self.nav_pub = self.create_publisher(String, '/robot/nav_command', 10)
        self.home_pub = self.create_publisher(String, '/robot/smarthome_command', 10)
        self.tts_pub = self.create_publisher(String, '/robot/speak', 10)
        
        self.voice_sub = self.create_subscription(String,'/robot/voice_command',self.voice_callback,10)
        
        self.home_state = {"ac": "off", "heater": "off", "light": "off", "tv": "off"}
        self.chat_history = []
        
        # Paths for config files
        self.prompts_path = os.path.expanduser('~/robot_ws/src/llm_control/config/languages.yaml')
        self.state_path = os.path.expanduser('~/robot_ws/src/llm_control/config/current_lang.yaml')
        self.telegram_contacts_path = os.path.expanduser('~/robot_ws/src/llm_control/config/telegram_contacts.yaml')
        self.telegram_config_path = os.path.expanduser('~/robot_ws/src/llm_control/config/telegram_config.yaml')
        self.telegram_messages_path = os.path.expanduser('~/robot_ws/src/llm_control/config/telegram_messages.yaml')
        
        # Ensure the messages file exists
        if not os.path.exists(self.telegram_messages_path):
            with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                yaml.dump([], f)

        # Load Languages
        with open(self.prompts_path, 'r', encoding='utf-8') as file:
            self.prompts = yaml.safe_load(file)
            
        # Load Telegram Contacts
        try:
            with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                self.telegram_contacts = yaml.safe_load(file) or {}
        except FileNotFoundError:
            self.get_logger().warning("telegram_contacts.yaml not found! Contacts won't load.")
            self.telegram_contacts = {}

        # Load Telegram Token
        try:
            with open(self.telegram_config_path, 'r', encoding='utf-8') as file:
                t_config = yaml.safe_load(file) or {}
                self.telegram_token = t_config.get('telegram_token', None)
        except FileNotFoundError:
            self.get_logger().warning("telegram_config.yaml not found! Telegram won't work.")
            self.telegram_token = None
            
        self.current_lang = "ar" 
        
        self.room_coordinates = {
            "office":  {"x": -5.584028244018555, "y": -3.163759469985962, "z": -0.001434326171875},
            "bedroom": {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
            "gym":     {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
            "kitchen": {"x": 6.558255195617676,  "y": -2.977426767349243, "z": 0.002532958984375},
            "living":  {"x": 0.8284147381782532, "y": -0.6738091707229614, "z": 0.002471923828125}
        }

        # --- START TELEGRAM LISTENER THREAD ---
        self.telegram_offset = 0
        if self.telegram_token:
            self.get_logger().info("📡 Starting Telegram polling thread (Inbox Mode)...")
            self.telegram_thread = threading.Thread(target=self.telegram_polling_loop, daemon=True)
            self.telegram_thread.start()
        
        self.get_logger().info("✅ Robot Ready! Connected to Koboldcpp with Multi-Language Memory & Telegram Inbox.")
    
    def telegram_polling_loop(self):
        """Checks for new messages and SAVES them to a YAML file instead of executing them."""
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
                                # Reverse lookup to find the sender's name
                                sender_name = "Unknown"
                                for name, cid in self.telegram_contacts.items():
                                    if str(cid) == chat_id:
                                        sender_name = name
                                        break
                                        
                                print(f"\n📩 [New Inbox Message from {sender_name}]: {text}")
                                
                                # Save the message to our unread YAML file
                                try:
                                    unread_msgs = []
                                    if os.path.exists(self.telegram_messages_path):
                                        with open(self.telegram_messages_path, 'r', encoding='utf-8') as f:
                                            unread_msgs = yaml.safe_load(f) or []
                                            
                                    unread_msgs.append({"sender": sender_name, "message": text})
                                    
                                    with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                                        yaml.dump(unread_msgs, f, allow_unicode=True)
                                except Exception as e:
                                    self.get_logger().error(f"Error saving telegram message: {e}")
                                    
                                print("\n🔵 Command: ", end="", flush=True)
                            else:
                                print(f"\n⚠️ [Unauthorized Telegram user]: ChatID {chat_id} said: '{text}'")
            except requests.exceptions.Timeout:
                pass 
            except Exception as e:
                self.get_logger().error(f"Telegram polling error: {e}")
                time.sleep(5)

    def voice_callback(self, msg):
        print(f"\n🎤 [Voice Detected]: {msg.data}")
        self.process_command(msg.data)
        print("\n🔵 Command: ", end="", flush=True)

    def extract_json(self, raw_text):
        clean_text = re.sub(r'<think>.*?</think>', '', raw_text, flags=re.DOTALL).strip()
        start = clean_text.find('{')
        end = clean_text.rfind('}') + 1
        if start != -1 and end != 0:
            return clean_text[start:end]
        return clean_text

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

    def process_command(self, text):
        self.get_logger().info(f"🧠 Processing with Qwen (Mode: {self.current_lang.upper()})...")
        
        base_prompt = self.prompts.get(self.current_lang, self.prompts["ar"])
        
        # Load Contacts and Unread Messages to inject into the LLM's brain
        available_contacts = list(self.telegram_contacts.keys())
        unread_messages = []
        if os.path.exists(self.telegram_messages_path):
            try:
                with open(self.telegram_messages_path, 'r', encoding='utf-8') as f:
                    unread_messages = yaml.safe_load(f) or []
            except:
                pass
                
        dynamic_prompt = base_prompt + f"\n\nCURRENT DEVICE STATES:\n{json.dumps(self.home_state)}\n\nAVAILABLE TELEGRAM CONTACTS:\n{json.dumps(available_contacts, ensure_ascii=False)}\n\nUNREAD TELEGRAM MESSAGES:\n{json.dumps(unread_messages, ensure_ascii=False)}"
        
        self.chat_history.append({"role": "user", "content": text})
        
        if len(self.chat_history) > 10:
            self.chat_history = self.chat_history[-10:]
            
        messages_payload = [{"role": "system", "content": dynamic_prompt}] + self.chat_history
        
        payload = {
            "messages": messages_payload,
            "temperature": 0.3,  
            "top_p": 0.7,
            "top_k": 20,
            "min_p": 0.0,
            "max_tokens": 500
        }

        try:
            response = requests.post(KOBOLD_URL, json=payload)
            result = response.json()
            raw_output = result['choices'][0]['message']['content']
            
            self.chat_history.append({"role": "assistant", "content": raw_output})
            json_string = self.extract_json(raw_output)
            data = json.loads(json_string)
            
            # --- READ INBOX LOGIC ---
            if data.get('intent') == 'read_telegram':
                try:
                    # Wipe the file clean since the LLM just read them out loud
                    with open(self.telegram_messages_path, 'w', encoding='utf-8') as f:
                        yaml.dump([], f)
                    print("📭 Inbox marked as read and cleared.")
                except Exception as e:
                    print(f"❌ Error clearing inbox: {e}")
            
            # --- TELEGRAM SENDING LOGIC ---
            elif data.get('intent') == 'telegram':
                contact_name = data.get('contact', '').lower()
                message_text = data.get('message', '')
                
                if contact_name in self.telegram_contacts:
                    target_chat_id = self.telegram_contacts[contact_name]
                    success = self.send_telegram_message(target_chat_id, message_text)
                    if success:
                        print(f"📱 TELEGRAM: Sent to {contact_name} -> {message_text}")
                    else:
                        print(f"❌ TELEGRAM Error: API request failed.")
                        data['response'] = "I'm sorry, I encountered an error sending the Telegram message."
                else:
                    print(f"❌ TELEGRAM Error: Contact '{contact_name}' not found in yaml.")
                    data['response'] = f"I cannot find a contact named {contact_name} in my database."

            # --- NAVIGATION LOGIC ---
            if data.get('navigate_to'):
                target_room = data['navigate_to']
                coords = self.room_coordinates.get(target_room, {"x": 0.0, "y": 0.0, "z": 0.0})
                nav_payload = {"intent": "navigate", "target": target_room, "action": "goto", "point": coords}
                msg = String()
                msg.data = json.dumps(nav_payload, ensure_ascii=False)
                self.nav_pub.publish(msg)
                print(f"🚀 NAV: {target_room}")

            # --- SMART HOME LOGIC ---
            if data.get('controls'):
                for ctrl in data['controls']:
                    if ctrl['target'] == 'language':
                        new_lang = ctrl['action']
                        if new_lang in self.prompts:
                            self.current_lang = new_lang
                            
                            with open(self.state_path, 'w') as f:
                                yaml.dump({"language": new_lang}, f)
                                
                            print(f"🌍 BRAIN SWITCHED TO LANGUAGE: {new_lang.upper()} (Saved to YAML)")
                    
                    elif ctrl['target'] in self.home_state:
                        self.home_state[ctrl['target']] = 'on' if ctrl['action'] == 'turn_on' else 'off'
                        print(f"🏠 HOME: {ctrl['action']} {ctrl['target']}")
                
                msg = String()
                msg.data = json.dumps({"controls": data['controls']}, ensure_ascii=False)
                self.home_pub.publish(msg)
            
            # --- SPEAK THE RESPONSE OUT LOUD ---
            if data.get('response'):
                response_text = data['response']
                tts_msg = String()
                tts_msg.data = response_text
                self.tts_pub.publish(tts_msg)
                print(f"🗣️ VOICE REPLY: {response_text}")

        except Exception as e:
            print(f"❌ Error: {e}")
            if self.chat_history: self.chat_history.pop()

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("\n🔵 Command: ")
                if user_input.strip():
                    self.process_command(user_input)
            except EOFError:
                break

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommander()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        node.input_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
