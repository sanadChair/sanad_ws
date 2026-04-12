#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import speech_recognition as sr
import os

class BulletproofWhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        
        self.publisher_ = self.create_publisher(String, '/robot/voice_command', 10)
        
        # We are no longer using the buggy 'stream' tool. We use the main 'cli' tool.
        self.whisper_bin = "/home/bod/whisper.cpp/build/bin/whisper-cli"
        self.model_path = os.path.expanduser("~/whisper.cpp/models/ggml-large-v3-q5_0.bin")
        self.temp_wav = "/tmp/user_voice.wav"
        
        self.wake_words = ["سند", "سناد", "صند", "sanad"]
        
        # Setup Professional Microphone Handling
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000  # Ignores quiet static
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.7    # Cuts off 0.7 seconds after you stop speaking
        
        self.get_logger().info("🎤 Pro Audio Node Ready. Calibrating microphone...")
        self.listen_loop()

    def listen_loop(self):
        with sr.Microphone() as source:
            # Listens to your room for 1 second to figure out how loud your PC fans are
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info("✅ Calibrated! Say 'سند' and your command...")
            
            while rclpy.ok():
                try:
                    # 1. Block and wait for actual human speech (Ignores silence)
                    audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=10)
                    
                    self.get_logger().info("Processing speech...")
                    
                    # 2. Save the spoken chunk to a temporary file
                    with open(self.temp_wav, "wb") as f:
                        f.write(audio.get_wav_data())
                    
                    # 3. Hand the file to the Whisper AI for instant transcription
                    # -nt means "no timestamps", so it outputs pure text
                    cmd = [
                        self.whisper_bin,
                        "-m", self.model_path,
                        "-f", self.temp_wav,
                        "-l", "ar",
                        "-nt"
                    ]
                    
                    # Run the command and capture the text output
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    output = result.stdout.strip()
                    
                    # Whisper often outputs blanks like [BLANK_AUDIO] if it heard a cough. Ignore them.
                    if output and "[" not in output:
                        self.get_logger().info(f"Raw Whisper heard: {output}")
                        
                        # 4. Check for the wake word
                        for wake in self.wake_words:
                            if wake in output:
                                parts = output.split(wake, 1)
                                if len(parts) > 1 and parts[1].strip():
                                    cmd_text = parts[1].strip()
                                    self.get_logger().info(f"🚀 Wake word triggered! Sending: {cmd_text}")
                                    msg = String()
                                    msg.data = cmd_text
                                    self.publisher_.publish(msg)
                                break
                                
                except Exception as e:
                    self.get_logger().error(f"Mic error: {e}")

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

if __name__ == '__main__':
    main()