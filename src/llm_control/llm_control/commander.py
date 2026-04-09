#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests
import threading
import time
import re

# --- KOBOLDCPP CONFIGURATION ---
KOBOLD_URL = "http://localhost:5001/v1/chat/completions"

BASE_PROMPT = """You are the brain of a smart home robot. You must ONLY output valid JSON. 

CRITICAL RULES:
1. The "response" MUST ALWAYS be in pure Arabic. Even if the user speaks English.
2. Devices must be strictly English: [ac, heater, light, tv].
3. Rooms must be strictly English: [kitchen, bedroom, living, office, gym, bathroom, entrance].
4. NEVER output <think> tags. Output your final JSON immediately.

INTENT MAPPING:
- Smart Home Commands (Hungry, Sleep, Leave, Hot, Cold, Work out, Train, etc.) -> Use "navigate" or "control".
  CRITICAL: For control/navigate, the "response" MUST be EXTREMELY SHORT (1 brief sentence). JUST confirm the action. DO NOT ask follow-up questions. DO NOT offer extra help.
- General Chat (Recipes, Questions, "how to") -> Use "chat". Provide a DETAILED, helpful Arabic answer.

EXAMPLES:
User: "i want to train"
{"intent": "navigate", "navigate_to": "gym", "controls": [], "response": "جاري التوجه إلى الصالة الرياضية."}

User: "how to make a cake"
{"intent": "chat", "navigate_to": null, "controls": [], "response": "لصنع كيكة لذيذة، اخلط 3 بيضات مع كوب سكر..."}

Strict JSON Format:
{
  "intent": "navigate" | "control" | "chat" | "navigate_and_control",
  "navigate_to": "room_name" | null,
  "controls": [
     {"target": "device_name", "action": "turn_on" | "turn_off"}
  ],
  "response": "Detailed Arabic reply for chat, VERY SHORT reply for commands"
}
"""

class LLMCommander(Node):
    def __init__(self):
        super().__init__('llm_commander')
        
        self.nav_pub = self.create_publisher(String, '/robot/nav_command', 10)
        self.home_pub = self.create_publisher(String, '/robot/smarthome_command', 10)
        self.tts_pub = self.create_publisher(String, '/robot/speak', 10)
        self.voice_sub = self.create_subscription(String,'/robot/voice_command',self.voice_callback,10)
        self.home_state = {"ac": "off", "heater": "off", "light": "off", "tv": "off"}
        
        # 🧠 CONVERSATIONAL MEMORY: Stores the chat history
        self.chat_history = []
        
        self.room_coordinates = {
            "office":  {"x": -5.584028244018555, "y": -3.163759469985962, "z": -0.001434326171875},
            "bedroom": {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
            "gym":     {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
            "kitchen": {"x": 6.558255195617676,  "y": -2.977426767349243, "z": 0.002532958984375},
            "living":  {"x": 0.8284147381782532, "y": -0.6738091707229614, "z": 0.002471923828125}
        }
        
        self.get_logger().info("✅ Robot Ready! Connected to Koboldcpp with Memory. Running in continuous mode.")
    
    def voice_callback(self, msg):
        self.get_logger().info(f"Mic heard: {msg.data}")
        self.process_command(msg.data)



    def extract_json(self, raw_text):
        clean_text = re.sub(r'<think>.*?</think>', '', raw_text, flags=re.DOTALL).strip()
        start = clean_text.find('{')
        end = clean_text.rfind('}') + 1
        if start != -1 and end != 0:
            return clean_text[start:end]
        return clean_text

    def process_command(self, text):
        self.get_logger().info("🧠 Processing with Qwen 3.5 9B...")
        
        dynamic_prompt = BASE_PROMPT + f"\n\nCURRENT DEVICE STATES:\n{json.dumps(self.home_state)}"
        
        # Add the new user message to the memory
        self.chat_history.append({"role": "user", "content": text})
        
        # Keep only the last 10 messages so we don't overflow the 4096 context window
        if len(self.chat_history) > 10:
            self.chat_history = self.chat_history[-10:]
            
        # Build the payload with the system prompt + chat history
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
            
            # Save the AI's response to the memory so it remembers what it said!
            self.chat_history.append({"role": "assistant", "content": raw_output})
            
            json_string = self.extract_json(raw_output)
            data = json.loads(json_string)
            
            if data.get('navigate_to'):
                target_room = data['navigate_to']
                coords = self.room_coordinates.get(target_room, {"x": 0.0, "y": 0.0, "z": 0.0})
                nav_payload = {"intent": "navigate", "target": target_room, "action": "goto", "point": coords}
                
                msg = String()
                msg.data = json.dumps(nav_payload, ensure_ascii=False)
                self.nav_pub.publish(msg)
                print(f"🚀 NAV: {target_room} -> X:{coords['x']:.2f}, Y:{coords['y']:.2f}")

            if data.get('controls'):
                for ctrl in data['controls']:
                    if ctrl['target'] in self.home_state:
                        self.home_state[ctrl['target']] = 'on' if ctrl['action'] == 'turn_on' else 'off'
                    print(f"🏠 HOME: {ctrl['action']} {ctrl['target']}")
                
                msg = String()
                msg.data = json.dumps({"controls": data['controls']}, ensure_ascii=False)
                self.home_pub.publish(msg)
            
            if data.get('response'):
                tts_msg = String()
                tts_msg.data = data['response']
                self.tts_pub.publish(tts_msg)
                print(f"🗣️ REPLY: {data.get('response')}")
            
            print(f"📊 STATE: AC({self.home_state['ac']}) | Heater({self.home_state['heater']})")

        except json.JSONDecodeError:
            print(f"❌ Error: Model did not output valid JSON. Raw output:\n{raw_output}")
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

if __name__ == '__main__':
    main()