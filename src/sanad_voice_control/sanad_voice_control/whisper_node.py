#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import speech_recognition as sr
import os
import re
import yaml

class BulletproofWhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        
        self.publisher_ = self.create_publisher(String, '/robot/voice_command', 10)
        
        # Path to the shared state file
        self.state_path = os.path.expanduser('~/robot_ws/src/llm_control/config/current_lang.yaml')
        
        self.wake_words = {
            "ar": ["سند", "سناد", "صند", "sanad"],
            "en": ["sanad", "senad", "sunad", "sonad", "sened","senate","samed"],
            "fr": ["sanad", "senad", "sanade", "sunad", "sonad", "sened","senate","samed","Sennad"],
            "de": ["sanad", "senad", "zanad", "sunad", "sonad", "sened","senate","samed","Sennad","Sennet"],
            "es": ["sanad", "senad", "sunad", "sonad", "sened","senate","samed","san","Sennad"],
            "it": ["sanad", "senad", "senna", "senat", "sunad", "sonad", "sened","senate","samed","Sennad"]
        }
        
        # Common Whisper hallucinations to ignore
        self.garbage_words = ["شكرا", "شكرا للمشاهدة", "ترجمة نانسي قنقر", "Thank you"]
        
        self.whisper_bin = "/home/bod/whisper.cpp/build/bin/whisper-cli"
        self.model_path = os.path.expanduser("~/whisper.cpp/models/ggml-large-v3-q5_0.bin")
        self.temp_wav = "/tmp/user_voice.wav"
        
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000  
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.7    
        
        self.get_logger().info("🎤 Pro Audio Node Ready. Calibrating microphone...")
        self.listen_loop()

    def get_current_language(self):
        # Always read the latest language from the file
        try:
            with open(self.state_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get("language", "ar")
        except Exception:
            return "ar"

    def is_hallucination(self, text):
        # Ignore exactly matching garbage phrases
        clean_text = text.strip().strip(".")
        return clean_text in self.garbage_words

    def listen_loop(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info("✅ Calibrated! Say your wake word...")
            
            while rclpy.ok():
                try:
                    # 1. Wait for audio
                    audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=10)
                    
                    # 2. Check the file to see what language we are currently in
                    current_lang = self.get_current_language()
                    
                    with open(self.temp_wav, "wb") as f:
                        f.write(audio.get_wav_data())
                    
                    # 3. Process with dynamic language
                    cmd = [
                        self.whisper_bin,
                        "-m", self.model_path,
                        "-f", self.temp_wav,
                        "-l", current_lang,
                        "-nt"
                    ]
                    
                    result = subprocess.run(cmd, capture_output=True, text=True)
                    output = result.stdout.strip()
                    
                    if output and "[" not in output and not self.is_hallucination(output):
                        self.get_logger().info(f"[{current_lang.upper()}] Heard: {output}")
                        
                        active_wakes = self.wake_words.get(current_lang, self.wake_words["ar"])
                        
                        for wake in active_wakes:
                            pattern = re.compile(re.escape(wake), re.IGNORECASE)
                            if pattern.search(output):
                                parts = pattern.split(output, 1)
                                if len(parts) > 1 and parts[1].strip():
                                    cmd_text = parts[1].strip()
                                    self.get_logger().info(f"🚀 Sending: {cmd_text}")
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
