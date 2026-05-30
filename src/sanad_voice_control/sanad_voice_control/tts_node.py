#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

class PiperTTSNode(Node):
    def __init__(self):
        super().__init__('piper_tts_node')
        
        self.declare_parameter('use_cloud', True)
        
        self.subscription = self.create_subscription(
            String,
            '/robot/speak',
            self.speak_callback,
            10)
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('sanad_voice_control')
            self.model_path = os.path.join(share_dir, 'voices', 'ar_JO-kareem-medium.onnx')
        except Exception:
            self.model_path = 'voices/ar_JO-kareem-medium.onnx'
        self.temp_audio_file = "/tmp/robot_speech.wav"
        
        # This variable keeps track of the active audio player
        self.audio_process = None
        
        self.get_logger().info("Arabic TTS Node Ready (Interruptible Mode). Listening for text...")

    def execute_local_piper(self, text_to_speak):
        try:
            # Generate the NEW audio file (This safely overwrites the old .wav file)
            generate_cmd = ['piper', '--model', self.model_path, '--output_file', self.temp_audio_file]
            subprocess.run(
                generate_cmd, 
                input=text_to_speak.encode('utf-8'),
                check=True, 
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.PIPE
            )
            
            # Play the audio ASYNCHRONOUSLY using Popen. 
            self.audio_process = subprocess.Popen(['aplay', '-q', self.temp_audio_file])
            
        except subprocess.CalledProcessError as e:
            error_details = e.stderr.decode('utf-8') if e.stderr else "Unknown Piper Error"
            self.get_logger().error(f"Piper crashed! Details: {error_details}")

    def speak_callback(self, msg):
        text_to_speak = msg.data
        
        # 1. INTERRUPT: Check if audio is currently playing. If yes, kill it immediately.
        if self.audio_process and self.audio_process.poll() is None:
            self.get_logger().info("Interrupting previous speech!")
            self.audio_process.terminate()
            self.audio_process.wait() # Wait a split second to ensure it cleanly stopped

        self.get_logger().info(f"Speaking: {text_to_speak}")
        
        use_cloud = self.get_parameter('use_cloud').get_parameter_value().bool_value
        
        if use_cloud:
            try:
                import wave
                from google import genai
                from google.genai import types
                
                self.get_logger().info("🌐 Calling Google Cloud Vertex AI (gemini-3.1-flash-tts-preview)...")
                client = genai.Client(
                    vertexai=True,
                    project="polar-ray-459012-b0",
                    location="us-central1",
                )
                
                response = client.models.generate_content(
                    model="gemini-3.1-flash-tts-preview",
                    contents=text_to_speak,
                    config=types.GenerateContentConfig(
                        response_modalities=["AUDIO"],
                        speech_config=types.SpeechConfig(
                            voice_config=types.VoiceConfig(
                                prebuilt_voice_config=types.PrebuiltVoiceConfig(
                                    voice_name="Puck"
                                )
                            )
                        ),
                    ),
                )
                
                audio_data = response.candidates[0].content.parts[0].inline_data.data
                
                # Save the PCM data to WAV
                with wave.open(self.temp_audio_file, "wb") as wf:
                    wf.setnchannels(1)
                    wf.setsampwidth(2)
                    wf.setframerate(24000)
                    wf.writeframes(audio_data)
                    
                # Play the audio ASYNCHRONOUSLY using Popen
                self.audio_process = subprocess.Popen(['aplay', '-q', self.temp_audio_file])
                
            except Exception as e:
                self.get_logger().error(f"❌ Google Cloud TTS failed: {e}. Falling back to local Piper.")
                self.execute_local_piper(text_to_speak)
        else:
            self.execute_local_piper(text_to_speak)

def main(args=None):
    rclpy.init(args=args)
    node = PiperTTSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Clean up the audio if you press Ctrl+C to shut down the node
        if node.audio_process and node.audio_process.poll() is None:
            node.audio_process.terminate()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()