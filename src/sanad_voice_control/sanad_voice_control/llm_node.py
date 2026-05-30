#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sanad_interfaces.srv import SpeakToLLM
import json
import requests
import os
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

        # ROS 2 Parameters
        self.declare_parameter("use_cloud", True)

        # ROS 2 Service Server
        self.srv = self.create_service(
            SpeakToLLM, "/robot/llm_query", self.handle_llm_query
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
            "✅ Stateless LLM Service Node is Ready! Serving /robot/llm_query"
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


def main(args=None):
    rclpy.init(args=args)
    node = LLMServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
