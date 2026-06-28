#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import speech_recognition as sr
import os
import yaml
import threading
import pyaudio
import numpy as np
from openwakeword.model import Model
import time
import re

class BulletproofWhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        
        self.publisher_ = self.create_publisher(String, '/robot/voice_command', 10)
        
        # Paths
        self.state_path = os.path.expanduser('~/robot_ws/src/llm_control/config/current_lang.yaml')
        self.oww_model_path = os.path.expanduser('~/robot_ws/src/llm_control/models/sanad.onnx')
        self.whisper_bin = "/home/bod/whisper.cpp/build/bin/whisper-cli"
        self.whisper_model = os.path.expanduser("~/whisper.cpp/models/ggml-large-v3-q5_0.bin")
        self.temp_wav = "/tmp/user_voice.wav"
        
        # Extended Whisper hallucinations to ignore
        self.garbage_words = [
            "شكرا", "شكرا للمشاهدة", "ترجمة نانسي قنقر", "Thank you", 
            "Amara.org", "Sous-titres", "Bye-bye", "I'm going to go to the next one"
        ]
        
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000  
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.7    
        
        self.get_logger().info(" Pro Audio Node Ready. Initializing openWakeWord...")
        
        # Run listening loop in a background thread so ROS 2 doesn't block
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

    def get_current_language(self):
        try:
            with open(self.state_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get("language", "ar")
        except Exception:
            return "ar"

    def is_hallucination(self, text):
        clean_text = text.strip()
        
        # If the text is just punctuation (like "-" or ".") block it
        if len(re.sub(r'\W+', '', clean_text)) == 0:
            return True
            
        for gw in self.garbage_words:
            if gw.lower() in clean_text.lower():
                return True
        return False

    def listen_loop(self):
        if not os.path.exists(self.oww_model_path):
            self.get_logger().error(f" Wake word model not found at: {self.oww_model_path}")
            return

        # Load your custom Sanad model
        oww_model = Model(wakeword_models=[self.oww_model_path], inference_framework="onnx")
        
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1280
        audio = pyaudio.PyAudio()
        
        self.get_logger().info(" openWakeWord Active! Say 'Sanad' to wake me up.")
        
        while rclpy.ok():
            try:
                # --- FIX 1: WIPE THE AI'S SHORT-TERM MEMORY ---
                oww_model.reset()
                
                # Open the microphone for the Wake Word Watchdog
                mic_stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
                
                # --- FIX 2: FLUSH THE OS AUDIO BUFFER ---
                # Throw away the first 5 chunks of audio to clear out lingering echoes
                for _ in range(5):
                    mic_stream.read(CHUNK, exception_on_overflow=False)
                    
                triggered = False
                
                while not triggered and rclpy.ok():
                    pcm = np.frombuffer(mic_stream.read(CHUNK, exception_on_overflow=False), dtype=np.int16)
                    prediction = oww_model.predict(pcm)
                    
                    for model_name, score in prediction.items():
                        if score > 0.6:  
                            print(f"\n WAKE WORD DETECTED! (Confidence: {score:.2f})")
                            triggered = True
                            break

                # CAREFULLY release the microphone so SpeechRecognition can use it
                mic_stream.stop_stream()
                mic_stream.close()
                time.sleep(0.1) # Tiny delay to ensure hardware is released
                
                # Move to the recording stage
                if triggered:
                    self.record_and_transcribe()
                    
            except Exception as e:
                self.get_logger().error(f"Audio stream error: {e}")
                time.sleep(1)

    def record_and_transcribe(self):
        current_lang = self.get_current_language()
        print(f"🎙️ Yes? Listening for your command in {current_lang.upper()}...")
        
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.2)
            try:
                # Wait for user to speak the actual command
                audio_data = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                
                with open(self.temp_wav, "wb") as f:
                    f.write(audio_data.get_wav_data())
                
                print(" Transcribing...")
                cmd = [
                    self.whisper_bin,
                    "-m", self.whisper_model,
                    "-f", self.temp_wav,
                    "-l", current_lang,
                    "-nt"
                ]
                
                result = subprocess.run(cmd, capture_output=True, text=True)
                output = result.stdout.strip()
                
                # Verify the output isn't empty or a hallucination
                if output and not self.is_hallucination(output):
                    print(f" Sending Command: {output}")
                    msg = String()
                    msg.data = output
                    self.publisher_.publish(msg)
                else:
                    print(" No valid command heard. Going back to sleep.")
                    
            except sr.WaitTimeoutError:
                print(" Timed out waiting for command. Going back to sleep.")
            except Exception as e:
                self.get_logger().error(f"Speech recognition error: {e}")

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
