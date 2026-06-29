#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import yaml
import re

class BulletproofWhisperNode(Node):
    def __init__(self):
        super().__init__("whisper_node")

        # Declare parameter for launch file compatibility
        self.declare_parameter("use_cloud", True)

        self.publisher_ = self.create_publisher(String, "/robot/voice_command", 10)

        # ROS 2 Subscriber for transcription requests
        self.subscription_ = self.create_subscription(
            String,
            "/whisper/transcribe_request",
            self.transcribe_callback,
            10
        )

        # Resolve paths dynamically
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("sanad_voice_control")
            self.state_path = os.path.join(share_dir, "config", "current_lang.yaml")
        except Exception:
            self.state_path = os.path.expanduser(
                "~/sanad_ws/src/sanad_voice_control/config/current_lang.yaml"
            )

        self.whisper_bin = "/home/bod/whisper.cpp/build/bin/whisper-cli"
        self.whisper_model = os.path.expanduser(
            "~/whisper.cpp/models/ggml-large-v3-q5_0.bin"
        )

        # Extended Whisper hallucinations to ignore
        self.garbage_words = [
            "شكرا",
            "شكرا للمشاهدة",
            "ترجمة نانسي قنقر",
            "Thank you",
            "Amara.org",
            "Sous-titres",
            "Bye-bye",
            "I'm going to go to the next one",
        ]

        self.get_logger().info("✅ Local Whisper Transcription Node Ready. Listening on /whisper/transcribe_request")

    def get_current_language(self):
        try:
            if os.path.exists(self.state_path):
                with open(self.state_path, "r") as f:
                    data = yaml.safe_load(f)
                    return data.get("language", "ar")
        except Exception:
            pass
        return "ar"

    def is_hallucination(self, text):
        clean_text = text.strip()

        # If the text is just punctuation (like "-" or ".") block it
        if len(re.sub(r"\W+", "", clean_text)) == 0:
            return True

        for gw in self.garbage_words:
            if gw.lower() in clean_text.lower():
                return True
        return False

    def transcribe_callback(self, msg):
        audio_path = msg.data
        if not os.path.exists(audio_path):
            self.get_logger().error(f"Audio file not found at: {audio_path}")
            return

        current_lang = self.get_current_language()
        self.get_logger().info(f"🎙️ Local transcribing {audio_path} in language: {current_lang.upper()}")

        try:
            cmd = [
                self.whisper_bin,
                "-m",
                self.whisper_model,
                "-f",
                audio_path,
                "-l",
                current_lang,
                "-nt",
            ]

            result = subprocess.run(cmd, capture_output=True, text=True)
            output = result.stdout.strip()

            # Verify the output isn't empty or a hallucination
            if output and not self.is_hallucination(output):
                self.get_logger().info(f"✅ Transcribed Command: {output}")
                out_msg = String()
                out_msg.data = output
                self.publisher_.publish(out_msg)
            else:
                self.get_logger().info("⚠️ No valid command heard or detected hallucination.")
        except Exception as e:
            self.get_logger().error(f"Error executing local whisper transcription: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BulletproofWhisperNode()

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
