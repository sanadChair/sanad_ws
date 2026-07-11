#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sanad_interfaces.srv import SpeakToLLM, CaptureAndDescribe, LocateObject
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import json
import requests
import os
import math
import struct
import threading
import tf2_ros
import tf2_geometry_msgs
from rclpy.callback_groups import ReentrantCallbackGroup
from google import genai
from google.genai import types


def load_dotenv():
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = get_package_share_directory("sanad_bringup")
        env_path = os.path.join(share_dir, ".env")
        if os.path.exists(env_path):
            with open(env_path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    if "=" in line:
                        k, v = line.split("=", 1)
                        os.environ[k.strip()] = v.strip().strip('"').strip("'")
    except Exception:
        pass


load_dotenv()

# --- LLM ENDPOINT CONFIGURATION ---
KOBOLD_URL = os.environ.get("KOBOLD_URL", "http://localhost:5001/v1/chat/completions")


class LLMServiceNode(Node):
    def __init__(self):
        super().__init__("llm_node")

        # ROS 2 Callback Group
        self.callback_group = ReentrantCallbackGroup()

        # ROS 2 Parameters
        self.declare_parameter("use_cloud", True)
        self.declare_parameter("camera_topic", "/realsense/image")
        self.declare_parameter("point_cloud_topic", "/realsense/points")

        # Vision variables
        self.latest_frame = None
        self.bridge = CvBridge()

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera Subscription
        camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10,
            callback_group=self.callback_group
        )

        # ROS 2 Service Servers
        self.srv = self.create_service(
            SpeakToLLM, "/robot/llm_query", self.handle_llm_query,
            callback_group=self.callback_group
        )
        self.capture_srv = self.create_service(
            CaptureAndDescribe, "/robot/capture_and_describe", self.handle_capture_and_describe,
            callback_group=self.callback_group
        )
        self.locate_srv = self.create_service(
            LocateObject, "/robot/locate_object", self.handle_locate_object,
            callback_group=self.callback_group
        )

        # Initialize Google GenAI client for Vertex AI
        try:
            self.genai_client = genai.Client(
                vertexai=True, project="polar-ray-459012-b0", location="us-central1"
            )
            self.get_logger().info("✅ Google GenAI Vertex AI client initialized successfully!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Google GenAI client: {e}")
            self.genai_client = None

        self.get_logger().info(
            "✅ Stateless LLM Service Node is Ready! Serving /robot/llm_query, /robot/capture_and_describe, and /robot/locate_object"
        )

    def handle_llm_query(self, request, response):
        text = request.command
        self.get_logger().info(f"🧠 Query received (length: {len(text)})")

        # Attempt to parse payload as a JSON-serialized messages array or dictionary
        try:
            payload_data = json.loads(text)
            if isinstance(payload_data, list):
                messages_payload = payload_data
            elif isinstance(payload_data, dict) and "messages" in payload_data:
                messages_payload = payload_data["messages"]
            else:
                messages_payload = [{"role": "user", "content": text}]
        except Exception:
            messages_payload = [{"role": "user", "content": text}]

        use_cloud = self.get_parameter("use_cloud").get_parameter_value().bool_value

        if use_cloud:
            if not self.genai_client:
                self.get_logger().error("❌ Google GenAI client is not initialized!")
                response.response = json.dumps(
                    {"response": "Error: Vertex AI client not initialized."}
                )
                return response

            self.get_logger().info(
                "🌐 Routing query to Cloud Gemini Vertex AI (model: gemini-2.5-flash)"
            )
            try:
                # Convert the messages_payload into Gemini format
                system_instruction = ""
                contents = []

                for msg in messages_payload:
                    role = msg.get("role")
                    content_str = msg.get("content", "")

                    if role == "system":
                        system_instruction = content_str
                        continue

                    gemini_role = "user" if role == "user" else "model"

                    # Check for audio command
                    if isinstance(content_str, str) and content_str.startswith("audio://"):
                        audio_path = content_str.replace("audio://", "")
                        if os.path.exists(audio_path):
                            try:
                                with open(audio_path, "rb") as f:
                                    audio_bytes = f.read()
                                
                                part = types.Part.from_bytes(
                                    data=audio_bytes,
                                    mime_type="audio/wav"
                                )
                                contents.append(types.Content(role=gemini_role, parts=[part]))
                                self.get_logger().info(f"🎤 Successfully attached audio from: {audio_path}")
                            except Exception as ex:
                                self.get_logger().error(f"❌ Failed to read audio file: {ex}")
                                contents.append(types.Content(role=gemini_role, parts=[types.Part.from_text(text="[Recorded Audio]")]))
                        else:
                            self.get_logger().error(f"❌ Audio file not found at path: {audio_path}")
                            contents.append(types.Content(role=gemini_role, parts=[types.Part.from_text(text="[Audio file not found]")]))
                    else:
                        contents.append(types.Content(role=gemini_role, parts=[types.Part.from_text(text=content_str)]))

                # Make sure we have a fallback system instruction if not provided
                if not system_instruction:
                    system_instruction = "أنت مساعد ذكي بيتكلم باللهجة المصرية العامية فقط. ردودك لازم تكون طبيعية وكأنك واحد من مصر."

                config_opts = types.GenerateContentConfig(system_instruction=system_instruction)
                
                gemini_res = self.genai_client.models.generate_content(
                    model="gemini-2.5-flash",
                    config=config_opts,
                    contents=contents,
                )
                
                raw_output = gemini_res.text
                response.response = raw_output
                self.get_logger().info(
                    f"✅ Cloud Gemini query complete! Returned {len(raw_output)} chars."
                )
            except Exception as e:
                self.get_logger().error(f"❌ Error during Cloud Gemini call: {e}")
                response.response = json.dumps(
                    {"response": "Error: Unable to reach Gemini Vertex AI backend."}
                )
        else:
            self.get_logger().info(f"🏠 Routing query to Local LLM ({KOBOLD_URL})")
            payload = {
                "messages": messages_payload,
                "temperature": 0.3,
                "top_p": 0.7,
                "top_k": 20,
                "min_p": 0.0,
                "max_tokens": 500,
            }

            try:
                res = requests.post(KOBOLD_URL, json=payload, timeout=20)
                res.raise_for_status()
                result = res.json()
                raw_output = result["choices"][0]["message"]["content"]

                response.response = raw_output
                self.get_logger().info(
                    f"✅ Local query complete! Returned {len(raw_output)} chars."
                )

            except Exception as e:
                self.get_logger().error(f"❌ Error during Koboldcpp call: {e}")
                response.response = json.dumps(
                    {"response": "Error: Unable to reach local LLM backend."}
                )

        return response

    def image_callback(self, msg):
        self.latest_frame = msg

    def handle_capture_and_describe(self, request, response):
        self.get_logger().info("📸 Capture and Describe service called!")
        
        if self.latest_frame is None:
            self.get_logger().error("❌ No image frame available from camera yet.")
            response.response = "Sorry, I can't see anything because the camera is not active or hasn't sent any frames."
            return response

        try:
            # Convert ROS Image to OpenCV BGR image
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_frame, desired_encoding="bgr8")
            # Encode OpenCV image to JPEG bytes
            success, encoded_img = cv2.imencode(".jpg", cv_img)
            if not success:
                raise Exception("Failed to encode image to JPEG")
            
            image_bytes = encoded_img.tobytes()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to process camera frame: {e}")
            response.response = "Sorry, I encountered an error while processing the camera image."
            return response

        if not self.genai_client:
            self.get_logger().error("❌ Gemini client not initialized")
            response.response = "Sorry, my visual brain client is not initialized."
            return response

        prompt = request.prompt
        if not prompt:
            prompt = "What do you see?"

        self.get_logger().info(f"🌐 Routing vision query to Gemini Vertex AI with prompt: {prompt}")

        try:
            # Prepare image part
            image_part = types.Part.from_bytes(
                data=image_bytes,
                mime_type="image/jpeg"
            )
            
            # Setup system instruction depending on prompt language
            system_instruction = "You are a helpful smart home robot. Describe what you see in the image clearly and concisely."
            if "شايف" in prompt or "ماذا" in prompt or "شوف" in prompt:
                system_instruction = "أنت مساعد ذكي بيتكلم باللهجة المصرية العامية فقط. صف ما تراه في الصورة بشكل طبيعي ومختصر وكأنك واحد من مصر."

            config_opts = types.GenerateContentConfig(system_instruction=system_instruction)
            
            # Call Gemini
            gemini_res = self.genai_client.models.generate_content(
                model="gemini-2.5-flash",
                config=config_opts,
                contents=[image_part, prompt],
            )
            
            raw_output = gemini_res.text
            response.response = raw_output
            self.get_logger().info(f"✅ Gemini response complete! Chars: {len(raw_output)}")
        except Exception as e:
            self.get_logger().error(f"❌ Error calling Gemini Vertex AI: {e}")
            response.response = "Sorry, I had trouble talking to my vision backend."

        return response

    def get_3d_point_from_cloud(self, cloud_msg, u, v):
        if cloud_msg.height <= 1:
            self.get_logger().warn("Point cloud is not organized, height <= 1")
            return None
        
        # Calculate offsets
        x_offset = y_offset = z_offset = None
        for field in cloud_msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
                
        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().error("Point cloud fields do not contain x, y, z")
            return None
            
        # Index in raw data
        index = v * cloud_msg.row_step + u * cloud_msg.point_step
        
        try:
            x = struct.unpack_from('f', cloud_msg.data, offset=index + x_offset)[0]
            y = struct.unpack_from('f', cloud_msg.data, offset=index + y_offset)[0]
            z = struct.unpack_from('f', cloud_msg.data, offset=index + z_offset)[0]
            return x, y, z
        except Exception as e:
            self.get_logger().error(f"Error unpacking point from cloud: {e}")
            return None

    def handle_locate_object(self, request, response):
        self.get_logger().info(f"🔍 Locate Object service called for: {request.object_name}")
        
        # 1. Capture the point cloud and color image
        cloud_event = threading.Event()
        captured_cloud = [None]
        
        def cloud_cb(msg):
            captured_cloud[0] = msg
            cloud_event.set()
            
        pc_topic = self.get_parameter("point_cloud_topic").get_parameter_value().string_value
        pc_sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            cloud_cb,
            10,
            callback_group=self.callback_group
        )
        
        # Wait up to 3.0 seconds for the point cloud
        success = cloud_event.wait(timeout=3.0)
        self.destroy_subscription(pc_sub)
        
        if not success or captured_cloud[0] is None:
            self.get_logger().error("❌ Failed to capture point cloud.")
            response.success = False
            return response
            
        cloud_msg = captured_cloud[0]
        
        # We also need the latest color frame (which we have from continuous subscription)
        if self.latest_frame is None:
            self.get_logger().error("❌ No color frame available.")
            response.success = False
            return response
            
        color_frame = self.latest_frame
        
        # Convert color frame to JPEG bytes
        try:
            cv_img = self.bridge.imgmsg_to_cv2(color_frame, desired_encoding="bgr8")
            success_enc, encoded_img = cv2.imencode(".jpg", cv_img)
            if not success_enc:
                raise Exception("JPEG encoding failed")
            image_bytes = encoded_img.tobytes()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to process color frame: {e}")
            response.success = False
            return response

        if not self.genai_client:
            self.get_logger().error("❌ Gemini client not initialized")
            response.success = False
            return response

        prompt = (
            f"Locate the {request.object_name} in the image. "
            "Return ONLY a JSON object with key 'box_2d' containing the normalized coordinates [ymin, xmin, ymax, xmax] "
            "scaled to 1000 (0 to 1000). For example: {'box_2d': [300, 400, 600, 800]}. "
            "If the object is not found or not visible, return {'box_2d': []}."
        )

        self.get_logger().info(f"🌐 Routing object detection to Gemini Vertex AI...")

        try:
            image_part = types.Part.from_bytes(
                data=image_bytes,
                mime_type="image/jpeg"
            )
            config_opts = types.GenerateContentConfig(
                system_instruction="You are an object detector. You must return ONLY the requested JSON structure and nothing else.",
                response_mime_type="application/json"
            )
            
            gemini_res = self.genai_client.models.generate_content(
                model="gemini-2.5-flash",
                config=config_opts,
                contents=[image_part, prompt],
            )
            
            result_text = gemini_res.text
            self.get_logger().info(f"Gemini object detection response: {result_text}")
            
            data = json.loads(result_text)
            box_2d = data.get("box_2d", [])
        except Exception as e:
            self.get_logger().error(f"❌ Error during Gemini query/parsing: {e}")
            response.success = False
            return response

        if not box_2d or len(box_2d) < 4:
            self.get_logger().warn(f"⚠️ Object '{request.object_name}' not found by Gemini.")
            response.success = False
            return response

        ymin, xmin, ymax, xmax = box_2d
        center_y_norm = (ymin + ymax) / 2.0
        center_x_norm = (xmin + xmax) / 2.0
        
        # Scale to point cloud resolution
        u = int(center_x_norm * cloud_msg.width / 1000.0)
        v = int(center_y_norm * cloud_msg.height / 1000.0)
        
        # Clip to cloud boundaries
        u = max(0, min(u, cloud_msg.width - 1))
        v = max(0, min(v, cloud_msg.height - 1))

        pt = self.get_3d_point_from_cloud(cloud_msg, u, v)
        if pt is None:
            self.get_logger().error("❌ Failed to lookup 3D point in cloud.")
            response.success = False
            return response
            
        x, y, z = pt
        self.get_logger().info(f"📍 Raw 3D point in camera frame: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Check if coordinates are valid
        if math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isinf(x) or math.isinf(y) or math.isinf(z):
            self.get_logger().warn("⚠️ Center point is NaN/Inf. Searching neighborhood...")
            found_valid = False
            for r in range(1, 15):
                for du in range(-r, r + 1):
                    for dv in range(-r, r + 1):
                        if abs(du) != r and abs(dv) != r:
                            continue
                        nu, nv = u + du, v + dv
                        if 0 <= nu < cloud_msg.width and 0 <= nv < cloud_msg.height:
                            npt = self.get_3d_point_from_cloud(cloud_msg, nu, nv)
                            if npt is not None:
                                nx, ny, nz = npt
                                if not (math.isnan(nx) or math.isnan(ny) or math.isnan(nz) or math.isinf(nx) or math.isinf(ny) or math.isinf(nz)):
                                    x, y, z = nx, ny, nz
                                    found_valid = True
                                    self.get_logger().info(f"✨ Found valid neighbor at offset ({du},{dv}): x={x:.3f}, y={y:.3f}, z={z:.3f}")
                                    break
                    if found_valid:
                        break
                if found_valid:
                    break
            
            if not found_valid:
                self.get_logger().error("❌ No valid 3D points in neighborhood.")
                response.success = False
                return response

        point_cam = PointStamped()
        point_cam.header.frame_id = cloud_msg.header.frame_id
        point_cam.header.stamp = cloud_msg.header.stamp
        point_cam.point.x = float(x)
        point_cam.point.y = float(y)
        point_cam.point.z = float(z)
        
        try:
            point_map = self.tf_buffer.transform(
                point_cam, "map", timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed, trying with latest available transform: {e}")
            try:
                point_cam.header.stamp = rclpy.time.Time().to_msg()
                point_map = self.tf_buffer.transform(
                    point_cam, "map", timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as ex:
                self.get_logger().error(f"❌ TF transformation to map frame failed: {ex}")
                response.success = False
                return response

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point_map.point.x
        pose.pose.position.y = point_map.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        response.pose = pose
        response.success = True
        self.get_logger().info(f"✅ Object located successfully in map frame: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
