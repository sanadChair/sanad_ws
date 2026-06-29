#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os
import yaml
import threading
import pyaudio
import numpy as np
from openwakeword.model import Model
import time

class WakeupNode(Node):
    def __init__(self):
        super().__init__("wakeup_node")

        # ROS 2 Parameters
        self.declare_parameter("use_cloud", True)

        # ROS 2 Publishers
        self.voice_cmd_pub = self.create_publisher(String, "/robot/voice_command", 10)
        self.whisper_req_pub = self.create_publisher(String, "/whisper/transcribe_request", 10)

        # Resolve paths dynamically
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory("sanad_voice_control")
            self.oww_model_path = os.path.join(share_dir, "models", "sanad.onnx")
            self.state_path = os.path.join(share_dir, "config", "current_lang.yaml")
            self.get_logger().info(f"Loaded config paths dynamically from share directory.")
        except Exception as e:
            self.get_logger().warn(f"Failed to get package share directory: {e}. Falling back to default paths.")
            self.oww_model_path = os.path.expanduser(
                "~/sanad_ws/src/sanad_voice_control/models/sanad.onnx"
            )
            self.state_path = os.path.expanduser(
                "~/sanad_ws/src/sanad_voice_control/config/current_lang.yaml"
            )

        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.7

        self.get_logger().info("🎙️ Wakeup Word Node Ready. Initializing openWakeWord...")

        # Run listening loop in a background thread so ROS 2 doesn't block
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

    def get_current_language(self):
        try:
            if os.path.exists(self.state_path):
                with open(self.state_path, "r") as f:
                    data = yaml.safe_load(f)
                    return data.get("language", "ar")
        except Exception as e:
            self.get_logger().warn(f"Failed to load language from {self.state_path}: {e}")
        return "ar"

    def listen_loop(self):
        if not os.path.exists(self.oww_model_path):
            self.get_logger().error(
                f"❌ Wake word model not found at: {self.oww_model_path}"
            )
            return

        # Load your custom Sanad model
        oww_model = Model(
            wakeword_models=[self.oww_model_path], inference_framework="onnx"
        )

        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        CHUNK = int(44100 * 0.08)  
        audio = pyaudio.PyAudio()

        self.get_logger().info(f"✅ openWakeWord Active! Model loaded from: {self.oww_model_path}. Say 'Sanad' to wake me up.")

        while rclpy.ok():
            try:
                # WIPE THE AI'S SHORT-TERM MEMORY
                oww_model.reset()

                # Open the microphone for the Wake Word Watchdog
                mic_stream = audio.open(
                    format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK,
                )

                # FLUSH THE OS AUDIO BUFFER
                # Throw away the first 5 chunks of audio to clear out lingering echoes
                for _ in range(5):
                    mic_stream.read(CHUNK, exception_on_overflow=False)

                triggered = False

                while not triggered and rclpy.ok():
                    pcm = np.frombuffer(
                        mic_stream.read(CHUNK, exception_on_overflow=False),
                        dtype=np.int16,
                    )
                    from scipy.signal import resample_poly

                    pcm16 = resample_poly(pcm, 16000, 44100).astype(np.int16)

                    prediction = oww_model.predict(pcm16)

                    for model_name, score in prediction.items():
                        if score > 0.3:
                            self.get_logger().info(f"🔊 WAKE WORD DETECTED! (Confidence: {score:.2f})")
                            print(f"\n WAKE WORD DETECTED! (Confidence: {score:.2f})")
                            triggered = True
                            break

                # CAREFULLY release the microphone so SpeechRecognition can use it
                mic_stream.stop_stream()
                mic_stream.close()
                time.sleep(0.1)  # Tiny delay to ensure hardware is released

                # Move to the recording stage
                if triggered:
                    self.record_and_dispatch()

            except Exception as e:
                self.get_logger().error(f"Audio stream error: {e}")
                time.sleep(1)

    def record_and_dispatch(self):
        current_lang = self.get_current_language()
        print(f"🎙️ Yes? Listening for your command in {current_lang.upper()}...")

        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            try:
                # Wait for user to speak the actual command
                audio_data = self.recognizer.listen(
                    source, timeout=5, phrase_time_limit=10
                )

                temp_wav = "/tmp/user_voice.wav"
                with open(temp_wav, "wb") as f:
                    f.write(audio_data.get_wav_data())

                use_cloud = self.get_parameter("use_cloud").get_parameter_value().bool_value
                
                if use_cloud:
                    self.get_logger().info("☁️ Dispatching command audio to Cloud (Gemini)...")
                    msg = String()
                    msg.data = f"audio://{temp_wav}"
                    self.voice_cmd_pub.publish(msg)
                else:
                    self.get_logger().info("🏠 Dispatching command audio to Local Whisper Node...")
                    msg = String()
                    msg.data = temp_wav
                    self.whisper_req_pub.publish(msg)

            except sr.WaitTimeoutError:
                print(" Timed out waiting for command. Going back to sleep.")
            except Exception as e:
                self.get_logger().error(f"Speech recognition error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WakeupNode()

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
