#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sanad_interfaces.srv import SpeakToLLM
import json
import requests

import os


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
            url = "https://inference.do-ai.run/v1/chat/completions"
            api_key = os.environ.get("DO_AI_API_KEY", "")
            if not api_key:
                self.get_logger().error(
                    "❌ DO_AI_API_KEY is not set in environment or .env file!"
                )
                response.response = json.dumps(
                    {"response": "Error: Cloud LLM API key not configured."}
                )
                return response

            headers = {
                "Content-Type": "application/json",
                "Authorization": api_key,
            }
            data = {
                "model": "alibaba-qwen3-32b",
                "messages": messages_payload,
                "max_tokens": 500,
            }

            self.get_logger().info(
                "🌐 Routing query to Cloud LLM (model: alibaba-qwen3-32b)"
            )
            try:
                res = requests.post(url, headers=headers, json=data, timeout=20)
                res.raise_for_status()
                result = res.json()
                raw_output = result["choices"][0]["message"]["content"]
                response.response = raw_output
                self.get_logger().info(
                    f"✅ Cloud query complete! Returned {len(raw_output)} chars."
                )
            except Exception as e:
                self.get_logger().error(f"❌ Error during Cloud LLM call: {e}")
                response.response = json.dumps(
                    {"response": "Error: Unable to reach Cloud LLM backend."}
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
