#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
import threading
import time
import re
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from sanad_interfaces.srv import NavigateToRoom

# --- KOBOLDCPP CONFIGURATION ---
KOBOLD_URL = "http://localhost:5001/v1/chat/completions"
llm_control_pkg = get_package_share_directory("llm_control")
sanad_room_manager_pkg = get_package_share_directory("sanad_room_manager")


def load_base_prompt() -> str:
    prompt_path = Path(llm_control_pkg) / "config" / "base_prompt.md"
    return prompt_path.read_text(encoding="utf-8").strip()


def load_rooms_yaml_text() -> str:
    rooms_path = Path(sanad_room_manager_pkg) / "config" / "rooms.yaml"
    try:
        return rooms_path.read_text(encoding="utf-8").strip()
    except Exception:
        return ""


BASE_PROMPT = load_base_prompt()
ROOMS_YAML_TEXT = load_rooms_yaml_text()


class LLMCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")

        self.home_pub = self.create_publisher(String, "/robot/smarthome_command", 10)
        self.tts_pub = self.create_publisher(String, "/robot/speak", 10)
        self.navigate_room_client = self.create_client(NavigateToRoom, "rooms/navigate")
        self.voice_sub = self.create_subscription(
            String, "/robot/voice_command", self.voice_callback, 10
        )
        self.home_state = {"ac": "off", "heater": "off", "light": "off", "tv": "off"}

        # 🧠 CONVERSATIONAL MEMORY: Stores the chat history
        self.chat_history = []

        if ROOMS_YAML_TEXT:
            self.get_logger().info("Loaded rooms.yaml text from share folder")
        else:
            self.get_logger().warn("Failed to load rooms.yaml text from share folder")

        self.get_logger().info(
            "✅ Robot Ready! Connected to Koboldcpp with Memory. Running in continuous mode."
        )

    def voice_callback(self, msg):
        self.get_logger().info(f"Mic heard: {msg.data}")
        self.process_command(msg.data)

    def _on_navigate_response(self, future, target_room):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(
                    f"Navigation service accepted room '{target_room}'"
                )
            else:
                self.get_logger().warn(
                    f"Navigation service failed for room '{target_room}'"
                )
        except Exception as exc:
            self.get_logger().error(
                f"Navigation service call exception for room '{target_room}': {exc}"
            )

    def navigate_to_room(self, target_room):
        if not self.navigate_room_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service 'rooms/navigate' is not available")
            return False

        request = NavigateToRoom.Request()
        request.room_name = target_room
        future = self.navigate_room_client.call_async(request)
        future.add_done_callback(lambda f: self._on_navigate_response(f, target_room))
        return True

    def extract_json(self, raw_text):
        clean_text = re.sub(
            r"<think>.*?</think>", "", raw_text, flags=re.DOTALL
        ).strip()
        start = clean_text.find("{")
        end = clean_text.rfind("}") + 1
        if start != -1 and end != 0:
            return clean_text[start:end]
        return clean_text

    def process_command(self, text):
        self.get_logger().info("🧠 Processing with Qwen 3.5 9B...")

        dynamic_prompt = (
            BASE_PROMPT + f"\n\nCURRENT DEVICE STATES:\n{json.dumps(self.home_state)}"
        )

        if ROOMS_YAML_TEXT:
            dynamic_prompt += f"\n\nROOMS YAML:\n{ROOMS_YAML_TEXT}"

        # Add the new user message to the memory
        self.chat_history.append({"role": "user", "content": text})

        # Keep only the last 10 messages so we don't overflow the 4096 context window
        if len(self.chat_history) > 10:
            self.chat_history = self.chat_history[-10:]

        # Build the payload with the system prompt + chat history
        messages_payload = [
            {"role": "system", "content": dynamic_prompt}
        ] + self.chat_history

        payload = {
            "messages": messages_payload,
            "temperature": 0.3,
            "top_p": 0.7,
            "top_k": 20,
            "min_p": 0.0,
            "max_tokens": 500,
        }

        try:
            response = requests.post(KOBOLD_URL, json=payload)
            result = response.json()

            raw_output = result["choices"][0]["message"]["content"]

            # Save the AI's response to the memory so it remembers what it said!
            self.chat_history.append({"role": "assistant", "content": raw_output})

            json_string = self.extract_json(raw_output)
            data = json.loads(json_string)

            if data.get("navigate_to"):
                target_room = data["navigate_to"]
                if self.navigate_to_room(target_room):
                    print(f"🚀 NAV: requested room '{target_room}' via rooms/navigate")

            if data.get("controls"):
                for ctrl in data["controls"]:
                    if ctrl["target"] in self.home_state:
                        self.home_state[ctrl["target"]] = (
                            "on" if ctrl["action"] == "turn_on" else "off"
                        )
                    print(f"🏠 HOME: {ctrl['action']} {ctrl['target']}")

                msg = String()
                msg.data = json.dumps(
                    {"controls": data["controls"]}, ensure_ascii=False
                )
                self.home_pub.publish(msg)

            if data.get("response"):
                tts_msg = String()
                tts_msg.data = data["response"]
                self.tts_pub.publish(tts_msg)
                print(f"🗣️ REPLY: {data.get('response')}")

            print(
                f"📊 STATE: AC({self.home_state['ac']}) | Heater({self.home_state['heater']})"
            )

        except json.JSONDecodeError:
            print(
                f"❌ Error: Model did not output valid JSON. Raw output:\n{raw_output}"
            )
            # If it fails, remove the bad response from memory so it doesn't get confused
            self.chat_history.pop()
        except Exception as e:
            print(f"❌ API Error: {e}")
            self.chat_history.pop()

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
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
