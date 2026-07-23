#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sanad_interfaces.srv import SpeakToLLM, NavigateToRoom, CaptureAndDescribe, LocateObject
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import json
import re
import os
import yaml
import math

class LLMCommander(Node):
    def __init__(self):
        super().__init__('llm_commander')
        
        # Publishers
        self.nav_pub = self.create_publisher(String, '/robot/nav_command', 10)
        self.home_pub = self.create_publisher(String, '/robot/smarthome_command', 10)
        self.tts_pub = self.create_publisher(String, '/robot/speak', 10)
        self.telegram_send_pub = self.create_publisher(String, '/robot/telegram/send', 10)
        self.telegram_clear_pub = self.create_publisher(String, '/robot/telegram/clear', 10)
        
        # Service Client for LLM Queries
        self.llm_client = self.create_client(
            SpeakToLLM,
            '/robot/llm_query'
        )
        
        # Service Client for Room Navigation
        self.nav_client = self.create_client(
            NavigateToRoom,
            'rooms/navigate'
        )

        # Service Client for Vision Analysis
        self.vision_client = self.create_client(
            CaptureAndDescribe,
            '/robot/capture_and_describe'
        )

        # Service Client for Dynamic Object Localization
        self.locate_client = self.create_client(
            LocateObject,
            '/robot/locate_object'
        )

        # Action Client for Arbitrary Nav Goals
        self.nav_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # TF Listener for robot localization
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        # Subscription to User Commands (Voice / Console)
        self.voice_sub = self.create_subscription(
            String,
            '/robot/voice_command',
            self.voice_callback,
            10
        )
        
        # State variables relocated from llm_node
        self.home_state = {"ac": "off", "heater": "off", "light": "off", "tv": "off"}
        self.chat_history = []
        self.current_lang = "ar"
        self.language_map = {
            "ar": "Arabic",
            "en": "English",
            "fr": "French",
            "de": "German",
            "es": "Spanish",
            "it": "Italian"
        }
        self.supported_languages = list(self.language_map.keys())

        # Paths for config files (resolved dynamically)
        self.prompts_path = self._resolve_config_path('system_prompt.yaml')
        self.state_path = self._resolve_config_path('current_lang.yaml')
        self.telegram_contacts_path = self._resolve_config_path('telegram_contacts.yaml')
        self.telegram_messages_path = self._resolve_config_path('telegram_messages.yaml')
        
        # Load language state at startup
        try:
            if os.path.exists(self.state_path):
                with open(self.state_path, 'r') as f:
                    data = yaml.safe_load(f)
                    self.current_lang = data.get("language", "ar")
        except Exception:
            pass

        # Load Languages Prompts
        try:
            with open(self.prompts_path, 'r', encoding='utf-8') as file:
                self.prompts = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load prompts: {e}")
            self.prompts = {}
            
        # Load Telegram Contacts
        try:
            with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                self.telegram_contacts = yaml.safe_load(file) or {}
        except FileNotFoundError:
            self.telegram_contacts = {}

        # Load dynamic room coordinates
        self.load_rooms_config()

        self.get_logger().info("✅ LLM Brain Orchestrator Node Ready! Service client connected.")

    def _robot_position(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf.transform.translation
        except Exception as e:
            self.get_logger().warn(f'TF robot position (map->base_link) failed: {e}', throttle_duration_sec=5.0)
            return None

    def _resolve_config_path(self, filename):
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sanad_voice_control')
            share_path = os.path.join(share_dir, 'config', filename)
            if os.path.exists(share_path):
                return share_path
        except Exception:
            pass
            
        return os.path.join('config', filename)

    def _resolve_rooms_yaml_path(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sanad_room_manager')
            share_yaml = os.path.join(share_dir, 'config', 'rooms.yaml')
            if os.path.isfile(share_yaml):
                return share_yaml
        except Exception:
            pass

        return None

    def load_rooms_config(self):
        # Initialize default fallback coordinates if not already present
        if not hasattr(self, 'room_coordinates') or not self.room_coordinates:
            self.room_coordinates = {
                "office":  {"x": -5.584028244018555, "y": -3.163759469985962, "z": -0.001434326171875},
                "bedroom": {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
                "gym":     {"x": -5.7033257484436035, "y": -0.234835684299469, "z": -0.001434326171875},
                "kitchen": {"x": 6.558255195617676,  "y": -2.977426767349243, "z": 0.002532958984375},
                "living":  {"x": 0.8284147381782532, "y": -0.6738091707229614, "z": 0.002471923828125}
            }
            
        rooms_yaml = self._resolve_rooms_yaml_path()
        if rooms_yaml and os.path.exists(rooms_yaml):
            try:
                with open(rooms_yaml, 'r', encoding='utf-8') as f:
                    rooms_data = yaml.safe_load(f) or {}
                
                # Check for "House" key just like room_manager_node
                if isinstance(rooms_data.get("House"), dict):
                    rooms_data = rooms_data["House"]
                
                if isinstance(rooms_data, dict):
                    for room_name, room_info in rooms_data.items():
                        if not isinstance(room_info, dict):
                            continue
                        coords = room_info.get("coordinates", {})
                        pose = coords.get("pose", {})
                        
                        x = float(pose.get("x", 0.0))
                        y = float(pose.get("y", 0.0))
                        z = float(pose.get("z", 0.0))
                        
                        self.room_coordinates[room_name] = {"x": x, "y": y, "z": z}
                        
                        # Add alias mappings for backwards compatibility
                        if room_name == "living_room":
                            self.room_coordinates["living"] = {"x": x, "y": y, "z": z}
                        elif room_name == "living":
                            self.room_coordinates["living_room"] = {"x": x, "y": y, "z": z}
                self.get_logger().info(f"Loaded coordinates from rooms.yaml: {list(self.room_coordinates.keys())}")
            except Exception as e:
                self.get_logger().error(f"Error loading rooms.yaml: {e}")
    
    def voice_callback(self, msg):
        self.get_logger().info(f"🎤 [Command Received]: {msg.data}")
        self.call_llm_service(msg.data)

    def call_llm_service(self, command_text):
        if not self.llm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Service /robot/llm_query not available! Is llm_node running?")
            return
            
        # Reload Languages Prompts dynamically just in case it was modified
        try:
            with open(self.prompts_path, 'r', encoding='utf-8') as file:
                self.prompts = yaml.safe_load(file) or {}
        except Exception:
            pass

        base_prompt_template = self.prompts.get("system_prompt", "")
        lang_name = self.language_map.get(self.current_lang, "Arabic")
        
        # Load available rooms dynamically from rooms.yaml
        rooms_list = ["kitchen", "living_room"] # fallback defaults
        rooms_yaml = self._resolve_rooms_yaml_path()
        if rooms_yaml and os.path.exists(rooms_yaml):
            try:
                with open(rooms_yaml, 'r', encoding='utf-8') as f:
                    rooms_data = yaml.safe_load(f) or {}
                if isinstance(rooms_data.get("House"), dict):
                    rooms_data = rooms_data["House"]
                if isinstance(rooms_data, dict):
                    parsed_rooms = list(rooms_data.keys())
                    if parsed_rooms:
                        rooms_list = parsed_rooms
            except Exception as e:
                self.get_logger().error(f"Error loading rooms.yaml: {e}")

        rooms_str = json.dumps(rooms_list, ensure_ascii=False)

        # Replace the placeholders {LANGUAGE_NAME}, {LANGUAGE_CODE}, and {ROOMS_LIST}
        base_prompt = base_prompt_template.replace("{LANGUAGE_NAME}", lang_name).replace("{LANGUAGE_CODE}", self.current_lang).replace("{ROOMS_LIST}", rooms_str)
        
        # Reload contacts and unread telegram messages dynamically in case they updated
        try:
            with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                self.telegram_contacts = yaml.safe_load(file) or {}
        except Exception:
            pass
            
        available_contacts = list(self.telegram_contacts.keys())
        unread_messages = []
        if os.path.exists(self.telegram_messages_path):
            try:
                with open(self.telegram_messages_path, 'r', encoding='utf-8') as f:
                    unread_messages = yaml.safe_load(f) or []
            except Exception:
                pass
                
        dynamic_prompt = base_prompt + f"\n\nCURRENT DEVICE STATES:\n{json.dumps(self.home_state)}\n\nAVAILABLE TELEGRAM CONTACTS:\n{json.dumps(available_contacts, ensure_ascii=False)}\n\nUNREAD TELEGRAM MESSAGES:\n{json.dumps(unread_messages, ensure_ascii=False)}"
        
        self.chat_history.append({"role": "user", "content": command_text})
        
        if len(self.chat_history) > 10:
            self.chat_history = self.chat_history[-10:]
            
        messages_payload = [{"role": "system", "content": dynamic_prompt}] + self.chat_history

        req = SpeakToLLM.Request()
        req.command = json.dumps(messages_payload, ensure_ascii=False)
        
        future = self.llm_client.call_async(req)
        future.add_done_callback(self.llm_callback)

    def extract_json(self, raw_text):
        clean_text = re.sub(r'<think>.*?</think>', '', raw_text, flags=re.DOTALL).strip()
        start = clean_text.find('{')
        end = clean_text.rfind('}') + 1
        if start != -1 and end != 0:
            return clean_text[start:end]
        return clean_text

    def llm_callback(self, future):
        try:
            response = future.result()
            raw_output = response.response
            
            self.chat_history.append({"role": "assistant", "content": raw_output})
            
            # Sanitize raw audio paths in chat history for future context
            for msg in self.chat_history:
                if msg.get("role") == "user" and isinstance(msg.get("content"), str) and msg["content"].startswith("audio://"):
                    msg["content"] = "[Voice Command]"
            
            # Update state variables internally based on LLM output
            try:
                json_string = self.extract_json(raw_output)
                data = json.loads(json_string)
                
                # Normalize control instructions into a list
                controls_list = []
                if data.get('intent') == 'control' and 'target' in data:
                    controls_list.append({
                        "target": data.get('target'),
                        "action": data.get('action'),
                        "room": data.get('room', '')
                    })
                elif data.get('controls'):
                    controls_list = data['controls']
                
                # Update Smart Home State and Language state
                for ctrl in controls_list:
                    target = ctrl.get('target')
                    action = str(ctrl.get('action'))
                    if target == 'language':
                        new_lang = action
                        if new_lang in self.supported_languages:
                            self.current_lang = new_lang
                            try:
                                with open(self.state_path, 'w') as f:
                                    yaml.dump({"language": new_lang}, f)
                            except Exception:
                                pass
                    elif target in self.home_state:
                        if action in ['on', 'turn_on']:
                            self.home_state[target] = 'on'
                        elif action in ['off', 'turn_off']:
                            self.home_state[target] = 'off'
                        else:
                            self.home_state[target] = action
            except Exception as e:
                self.get_logger().warn(f"Failed to parse LLM json for state updates: {e}")

            # Extract json context again for routing actuators
            try:
                json_string = self.extract_json(raw_output)
                data = json.loads(json_string)
            except Exception:
                data = {}
            
            # --- READ INBOX LOGIC ---
            if data.get('intent') == 'read_telegram':
                scope = data.get('read_scope', 'all')
                clear_payload = {}
                if scope == 'all' or not scope:
                    clear_payload = {"scope": "all"}
                else:
                    clear_payload = {"scope": "sender", "sender": scope}
                
                clear_msg = String()
                clear_msg.data = json.dumps(clear_payload)
                self.telegram_clear_pub.publish(clear_msg)
            
            # --- TELEGRAM SENDING LOGIC ---
            elif data.get('intent') == 'telegram':
                contact_name = data.get('contact', '').lower()
                message_text = data.get('message', '')
                
                # Reload contacts dynamically
                try:
                    with open(self.telegram_contacts_path, 'r', encoding='utf-8') as file:
                        self.telegram_contacts = yaml.safe_load(file) or {}
                except Exception:
                    pass

                if contact_name in self.telegram_contacts:
                    telegram_payload = {"contact": contact_name, "message": message_text}
                    send_msg = String()
                    send_msg.data = json.dumps(telegram_payload)
                    self.telegram_send_pub.publish(send_msg)
                else:
                    self.get_logger().error(f"❌ TELEGRAM Error: Contact '{contact_name}' not found.")
                    data['response'] = f"I cannot find a contact named {contact_name} in my database."

            # --- VISION LOGIC ---
            elif data.get('intent') == 'vision':
                if self.vision_client.wait_for_service(timeout_sec=2.0):
                    req = CaptureAndDescribe.Request()
                    
                    # Extract the question parsed by the LLM
                    user_question = data.get('question')
                    if user_question:
                        req.prompt = user_question
                        self.get_logger().info(f"📸 Specific vision question extracted: {req.prompt}")
                    else:
                        # Fallback to active language generic prompts
                        prompts_map = {
                            "ar": "ماذا ترى أمامك؟ صف بالتفصيل باللهجة المصرية العامية.",
                            "en": "What do you see in front of you? Describe it in detail.",
                            "fr": "Que vois-tu devant toi ? Décris-le en détail.",
                            "de": "Was siehst du vor dir? Beschreibe es im Detail.",
                            "es": "¿Qué ves delante de ti? Descríbelo en detail.",
                            "it": "Cosa vedi davanti a te? Descrivilo in dettaglio."
                        }
                        req.prompt = prompts_map.get(self.current_lang, "What do you see in front of you? Describe it in detail.")
                        self.get_logger().info(f"📸 Generic vision request prompt: {req.prompt}")
                    
                    future_vision = self.vision_client.call_async(req)
                    
                    def vision_done_callback(fut):
                        try:
                            res = fut.result()
                            vision_desc = res.response
                            self.get_logger().info(f"✅ Vision analysis complete: {vision_desc}")
                            
                            # Append to chat history so the assistant remembers what it saw
                            self.chat_history.append({"role": "assistant", "content": vision_desc})
                            
                            # Speak the description
                            tts_msg = String()
                            tts_msg.data = vision_desc
                            self.tts_pub.publish(tts_msg)
                            self.get_logger().info(f"🗣️ Vision Response Published to TTS: {vision_desc}")
                        except Exception as ex:
                            self.get_logger().error(f"❌ Vision service call failed: {ex}")
                            
                    future_vision.add_done_callback(vision_done_callback)
                else:
                    self.get_logger().error("❌ Service /robot/capture_and_describe not available!")
                    fallback_response = "Sorry, my camera service is not available."
                    tts_msg = String()
                    tts_msg.data = fallback_response
                    self.tts_pub.publish(tts_msg)

            # --- VISION-BASED OBJECT NAVIGATION LOGIC ---
            elif data.get('intent') == 'navigate_to_object':
                target_obj = data.get('object', '')
                if not target_obj:
                    self.get_logger().error("❌ NAV OBJECT Error: Object name is empty.")
                else:
                    if self.locate_client.wait_for_service(timeout_sec=2.0):
                        req = LocateObject.Request()
                        req.object_name = target_obj
                        
                        self.get_logger().info(f"🔍 Requesting location for object: {target_obj}")
                        future_locate = self.locate_client.call_async(req)
                        
                        def locate_done_callback(fut):
                            try:
                                res = fut.result()
                                if not res.success:
                                    self.get_logger().error(f"❌ Failed to locate object '{target_obj}'")
                                    error_msg = "Sorry, I couldn't find that object in my view."
                                    if self.current_lang == "ar":
                                        error_msg = "أنا آسف، مش شايف الحاجة دي قدامي."
                                    tts_msg = String()
                                    tts_msg.data = error_msg
                                    self.tts_pub.publish(tts_msg)
                                    return
                                    
                                obj_pose = res.pose
                                self.get_logger().info(f"📍 Object located at: x={obj_pose.pose.position.x:.3f}, y={obj_pose.pose.position.y:.3f}")
                                
                                # Get robot current position
                                robot_pos = self._robot_position()
                                if robot_pos is None:
                                    self.get_logger().error("❌ Failed to get robot position.")
                                    return
                                    
                                # Compute offset goal (keep 0.8m distance)
                                dx = obj_pose.pose.position.x - robot_pos.x
                                dy = obj_pose.pose.position.y - robot_pos.y
                                dist = math.hypot(dx, dy)
                                
                                min_dist = 0.8 # keep 0.8 meters distance
                                
                                if dist < min_dist:
                                    self.get_logger().info("Already close enough to the object.")
                                    success_msg = "I am already near the object."
                                    if self.current_lang == "ar":
                                        success_msg = "أنا قريب من الحاجة دي بالفعل."
                                    tts_msg = String()
                                    tts_msg.data = success_msg
                                    self.tts_pub.publish(tts_msg)
                                    return
                                    
                                scale = (dist - min_dist) / dist
                                
                                goal_pose = PoseStamped()
                                goal_pose.header.frame_id = "map"
                                goal_pose.header.stamp = self.get_clock().now().to_msg()
                                goal_pose.pose.position.x = robot_pos.x + dx * scale
                                goal_pose.pose.position.y = robot_pos.y + dy * scale
                                goal_pose.pose.position.z = 0.0
                                
                                # Face the object
                                yaw = math.atan2(dy, dx)
                                goal_pose.pose.orientation.x = 0.0
                                goal_pose.pose.orientation.y = 0.0
                                goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
                                goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
                                
                                self.get_logger().info(f"🚀 Sending Nav2 goal to Pose: x={goal_pose.pose.position.x:.3f}, y={goal_pose.pose.position.y:.3f}")
                                
                                # Call NavigateToPose action
                                if self.nav_pose_client.wait_for_server(timeout_sec=5.0):
                                    goal_msg = NavigateToPose.Goal()
                                    goal_msg.pose = goal_pose
                                    
                                    send_goal_future = self.nav_pose_client.send_goal_async(goal_msg)
                                    
                                    def goal_response_callback(future_accepted):
                                        goal_handle = future_accepted.result()
                                        if not goal_handle.accepted:
                                            self.get_logger().error("Nav2 rejected goal")
                                            return
                                        self.get_logger().info("Nav2 goal accepted")
                                        
                                    send_goal_future.add_done_callback(goal_response_callback)
                                else:
                                    self.get_logger().error("NavigateToPose action server not available")
                                    
                            except Exception as ex:
                                self.get_logger().error(f"❌ Error during locate object callback: {ex}")
                                
                        future_locate.add_done_callback(locate_done_callback)
                    else:
                        self.get_logger().error("❌ Service /robot/locate_object not available!")
                        fallback_response = "Sorry, my visual locator service is not available."
                        if self.current_lang == "ar":
                            fallback_response = "أنا آسف، خدمة تحديد المواقع البصرية مش متاحة حالياً."
                        tts_msg = String()
                        tts_msg.data = fallback_response
                        self.tts_pub.publish(tts_msg)

            # --- NAVIGATION LOGIC ---
            if data.get('navigate_to'):
                target_room = data['navigate_to']
                # Reload dynamic rooms config to pick up runtime updates instantly
                self.load_rooms_config()
                coords = self.room_coordinates.get(target_room, {"x": 0.0, "y": 0.0, "z": 0.0})
                nav_payload = {"intent": "navigate", "target": target_room, "action": "goto", "point": coords}
                msg = String()
                msg.data = json.dumps(nav_payload, ensure_ascii=False)
                self.nav_pub.publish(msg)
                self.get_logger().info(f"🚀 NAV Command Published: {target_room}")

                # Call rooms/navigate service to actually command the robot via Nav2
                if self.nav_client.wait_for_service(timeout_sec=2.0):
                    req = NavigateToRoom.Request()
                    req.room_name = target_room
                    self.get_logger().info(f"🚀 Requesting navigation to room: {target_room} via rooms/navigate service")
                    future = self.nav_client.call_async(req)
                    
                    def nav_done_callback(fut):
                        try:
                            res = fut.result()
                            if res.success:
                                self.get_logger().info(f"✅ Navigation to {target_room} succeeded.")
                            else:
                                self.get_logger().error(f"❌ Navigation to {target_room} failed.")
                        except Exception as ex:
                            self.get_logger().error(f"❌ Navigation service call failed: {ex}")
                            
                    future.add_done_callback(nav_done_callback)
                else:
                    self.get_logger().error("❌ Service rooms/navigate not available!")

            # --- SMART HOME LOGIC ---
            controls_list = []
            if data.get('intent') == 'control' and 'target' in data:
                controls_list.append({
                    "target": data.get('target'),
                    "action": data.get('action'),
                    "room": data.get('room', '')
                })
            elif data.get('controls'):
                controls_list = data['controls']
                
            if controls_list:
                msg = String()
                msg.data = json.dumps({"controls": controls_list}, ensure_ascii=False)
                self.home_pub.publish(msg)
                for ctrl in controls_list:
                    self.get_logger().info(f"🏠 HOME Command Processed: {ctrl.get('action')} {ctrl.get('target')}")
            
            # --- SPEAK THE RESPONSE OUT LOUD ---
            if data.get('response'):
                response_text = data['response']
                tts_msg = String()
                tts_msg.data = response_text
                self.tts_pub.publish(tts_msg)
                self.get_logger().info(f"🗣️ TTS Response Published: {response_text}")

        except Exception as e:
            self.get_logger().error(f"❌ Error in LLM service call response callback: {e}")
            if self.chat_history:
                self.chat_history.pop()

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()
