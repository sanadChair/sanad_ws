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
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sanad_voice_control')
            self.state_path = os.path.join(share_dir, 'config', 'current_lang.yaml')
        except Exception:
            self.state_path = 'config/current_lang.yaml'
        
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
        
        # ROS 2 Parameters
        self.declare_parameter('use_cloud', True)
        self.declare_parameter('key_trigger', 'p')
        
        # PTT State Variables
        self.is_recording = False
        self.recording_frames = []
        self.recording_thread = None
        self.continuous_listener_active = True
        
        # Setup pynput Listener
        try:
            from pynput import keyboard
            self.pynput_listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release
            )
            self.pynput_listener.start()
            self.get_logger().info("🎹 Global Keyboard PTT Listener active. Hold down 'p' to speak!")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Global Keyboard Listener failed to start: {e}. Fallback to continuous wake word mode.")
        
        # Run continuous listener in background thread to avoid blocking spin()
        import threading
        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()
        
        self.get_logger().info("🎤 Pro Audio Node Ready. Ambient continuous listening enabled.")

    def on_press(self, key):
        try:
            trigger_char = self.get_parameter('key_trigger').get_parameter_value().string_value
            if hasattr(key, 'char') and key.char == trigger_char:
                if not self.is_recording:
                    self.get_logger().info(f"🔴 PTT Active: Key '{trigger_char}' held. Recording voice...")
                    self.is_recording = True
                    self.continuous_listener_active = False # Pause continuous audio
                    self.recording_frames = []
                    
                    import threading
                    self.recording_thread = threading.Thread(target=self.record_audio)
                    self.recording_thread.start()
        except Exception as e:
            self.get_logger().error(f"Error in PTT press: {e}")

    def on_release(self, key):
        try:
            trigger_char = self.get_parameter('key_trigger').get_parameter_value().string_value
            if hasattr(key, 'char') and key.char == trigger_char:
                if self.is_recording:
                    self.get_logger().info(f"🟩 PTT Released: Key '{trigger_char}' released. Processing...")
                    self.is_recording = False
                    if self.recording_thread:
                        self.recording_thread.join()
                        self.recording_thread = None
                    
                    import threading
                    processing_thread = threading.Thread(target=self.process_ptt_audio)
                    processing_thread.start()
        except Exception as e:
            self.get_logger().error(f"Error in PTT release: {e}")

    def record_audio(self):
        try:
            import pyaudio
            p = pyaudio.PyAudio()
            stream = p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=1024
            )
            
            while self.is_recording and rclpy.ok():
                data = stream.read(1024, exception_on_overflow=False)
                self.recording_frames.append(data)
                
            stream.stop_stream()
            stream.close()
            p.terminate()
        except Exception as e:
            self.get_logger().error(f"PyAudio recording error: {e}")

    def process_ptt_audio(self):
        try:
            if not self.recording_frames:
                self.get_logger().warn("⚠️ Recorded empty speech.")
                self.continuous_listener_active = True
                return
                
            import wave
            with wave.open(self.temp_wav, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(b''.join(self.recording_frames))
                
            use_cloud = self.get_parameter('use_cloud').get_parameter_value().bool_value
            
            if use_cloud:
                transcript = self.transcribe_google_cloud(self.temp_wav)
            else:
                transcript = self.transcribe_local_whisper(self.temp_wav)
                
            if transcript:
                self.get_logger().info(f"🗣️ Transcribed Command: '{transcript}'")
                # Direct publishing (PTT bypasses wake-word)
                msg = String()
                msg.data = transcript
                self.publisher_.publish(msg)
                self.get_logger().info("🚀 Sent PTT Command directly!")
                
        except Exception as e:
            self.get_logger().error(f"PTT processing failed: {e}")
        finally:
            self.continuous_listener_active = True

    def transcribe_google_cloud(self, audio_file_path):
        self.get_logger().info("🌐 Calling Google Cloud Speech-to-Text...")
        try:
            from google.cloud import speech
            client = speech.SpeechClient()
            
            with open(audio_file_path, "rb") as f:
                content = f.read()
                
            audio = speech.RecognitionAudio(content=content)
            config = speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code="ar-EG",  # Egyptian dialect
            )
            
            response = client.recognize(config=config, audio=audio)
            
            transcripts = []
            for result in response.results:
                transcripts.append(result.alternatives[0].transcript)
                
            if transcripts:
                return " ".join(transcripts).strip()
            self.get_logger().warn("⚠️ Google Cloud returned empty transcript.")
            return None
        except Exception as e:
            self.get_logger().error(f"❌ Google Cloud STT failed: {e}")
            return None

    def transcribe_local_whisper(self, audio_file_path):
        self.get_logger().info("🏠 Fallback to local Whisper.cpp...")
        try:
            current_lang = self.get_current_language()
            cmd = [
                self.whisper_bin,
                "-m", self.model_path,
                "-f", audio_file_path,
                "-l", current_lang,
                "-nt"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            output = result.stdout.strip()
            if output and not self.is_hallucination(output):
                output = re.sub(r'\[.*?\]', '', output).strip()
                return output
            return None
        except Exception as e:
            self.get_logger().error(f"❌ Local Whisper PTT failed: {e}")
            return None

    def get_current_language(self):
        try:
            with open(self.state_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get("language", "ar")
        except Exception:
            return "ar"

    def is_hallucination(self, text):
        clean_text = text.strip().strip(".")
        return clean_text in self.garbage_words

    def listen_loop(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            self.get_logger().info("✅ Ambient mic calibrated!")
            
            while rclpy.ok():
                if not self.continuous_listener_active:
                    import time
                    time.sleep(0.1)
                    continue
                    
                try:
                    # Non-blocking listen to allow quick PTT interruptions
                    audio = self.recognizer.listen(source, timeout=2.0, phrase_time_limit=10)
                    
                    if not self.continuous_listener_active:
                        continue
                        
                    current_lang = self.get_current_language()
                    
                    with open(self.temp_wav, "wb") as f:
                        f.write(audio.get_wav_data())
                    
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
                        self.get_logger().info(f"[{current_lang.upper()}] Ambient Heard: {output}")
                        
                        active_wakes = self.wake_words.get(current_lang, self.wake_words["ar"])
                        
                        for wake in active_wakes:
                            pattern = re.compile(re.escape(wake), re.IGNORECASE)
                            if pattern.search(output):
                                parts = pattern.split(output, 1)
                                if len(parts) > 1 and parts[1].strip():
                                    cmd_text = parts[1].strip()
                                    self.get_logger().info(f"🚀 Ambient Sending: {cmd_text}")
                                    msg = String()
                                    msg.data = cmd_text
                                    self.publisher_.publish(msg)
                                break
                except sr.WaitTimeoutError:
                    continue
                except Exception as e:
                    self.get_logger().error(f"Ambient mic error: {e}")

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
