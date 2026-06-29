#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import threading
import queue
import pyaudio
import wave


class AudioPlayer:
    def __init__(self, node):
        self.node = node
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.is_playing = False
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def stop(self):
        with self.lock:
            self.stop_event.set()
            if self.stream:
                try:
                    self.stream.stop_stream()
                    self.stream.close()
                except Exception:
                    pass
                self.stream = None
            self.is_playing = False

    def play_wav(self, file_path):
        self.stop()
        with self.lock:
            self.stop_event.clear()
            self.is_playing = True

        def run():
            try:
                wf = wave.open(file_path, "rb")
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to open WAV file {file_path}: {e}"
                )
                self.is_playing = False
                return

            with self.lock:
                if self.stop_event.is_set():
                    wf.close()
                    self.is_playing = False
                    return
                try:
                    self.stream = self.p.open(
                        format=self.p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True,
                    )
                except Exception as e:
                    self.node.get_logger().error(
                        f"Failed to open pyaudio output stream: {e}"
                    )
                    wf.close()
                    self.is_playing = False
                    return

            chunk_size = 1024
            data = wf.readframes(chunk_size)
            while data and not self.stop_event.is_set():
                try:
                    self.stream.write(data)
                except Exception as e:
                    self.node.get_logger().error(
                        f"Error writing to audio stream: {e}"
                    )
                    break
                data = wf.readframes(chunk_size)

            wf.close()
            self.stop()

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def play_stream(self, rate=24000, channels=1):
        self.stop()
        with self.lock:
            self.stop_event.clear()
            self.is_playing = True
            self.audio_queue = queue.Queue()
            try:
                self.stream = self.p.open(
                    format=pyaudio.paInt16,
                    channels=channels,
                    rate=rate,
                    output=True,
                )
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to open streaming pyaudio output stream: {e}"
                )
                self.is_playing = False
                return

        def run():
            while not self.stop_event.is_set():
                try:
                    # block with a timeout to check stop_event periodically
                    chunk = self.audio_queue.get(timeout=0.1)
                    if chunk is None:  # Sentinel to finish
                        break
                    with self.lock:
                        if self.stop_event.is_set():
                            break
                        if self.stream:
                            self.stream.write(chunk)
                except queue.Empty:
                    continue
                except Exception as e:
                    self.node.get_logger().error(
                        f"Error writing streaming chunk to audio: {e}"
                    )
                    break
            self.stop()

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def write_stream_chunk(self, chunk):
        if self.is_playing and hasattr(self, "audio_queue"):
            self.audio_queue.put(chunk)

    def finalize_stream(self):
        if self.is_playing and hasattr(self, "audio_queue"):
            self.audio_queue.put(None)


class PiperTTSNode(Node):
    def __init__(self):
        super().__init__("piper_tts_node")

        self.declare_parameter("use_cloud", True)

        self.subscription = self.create_subscription(
            String, "/robot/speak", self.speak_callback, 10
        )
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory("sanad_voice_control")
            self.model_path = os.path.join(
                share_dir, "voices", "ar_JO-kareem-medium.onnx"
            )
        except Exception:
            self.model_path = "voices/ar_JO-kareem-medium.onnx"
        self.temp_audio_file = "/tmp/robot_speech.wav"

        # This player manages thread-safe interruptible audio playback via PyAudio
        self.audio_player = AudioPlayer(self)

        self.get_logger().info(
            "Arabic TTS Node Ready (Interruptible Mode). Listening for text..."
        )

    def execute_local_piper(self, text_to_speak):
        try:
            # Generate the NEW audio file (This safely overwrites the old .wav file)
            generate_cmd = [
                "piper",
                "--model",
                self.model_path,
                "--output_file",
                self.temp_audio_file,
            ]
            subprocess.run(
                generate_cmd,
                input=text_to_speak.encode("utf-8"),
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )

            # Play the audio using PyAudio
            self.audio_player.play_wav(self.temp_audio_file)

        except subprocess.CalledProcessError as e:
            error_details = (
                e.stderr.decode("utf-8") if e.stderr else "Unknown Piper Error"
            )
            self.get_logger().error(f"Piper crashed! Details: {error_details}")

    def speak_callback(self, msg):
        text_to_speak = msg.data

        # 1. INTERRUPT: Stop any current speech playback immediately.
        self.audio_player.stop()

        self.get_logger().info(f"Speaking: {text_to_speak}")

        use_cloud = (
            self.get_parameter("use_cloud").get_parameter_value().bool_value
        )

        if use_cloud:
            try:
                from google import genai
                from google.genai import types

                self.get_logger().info("🌐 Streaming Google Cloud Vertex AI...")
                client = genai.Client(
                    vertexai=True,
                    project="polar-ray-459012-b0",
                    location="us-central1",
                )

                # Start PyAudio player in streaming mode (24kHz Mono 16-bit)
                self.audio_player.play_stream(rate=24000, channels=1)

                # 2. Use the streaming variant of the API
                response_stream = client.models.generate_content_stream(
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

                # 3. Pipe chunks directly into PyAudio player as they arrive
                for chunk in response_stream:
                    # Check if another process interrupted this speech mid-stream
                    if not self.audio_player.is_playing:
                        break

                    try:
                        if (
                            chunk.candidates
                            and chunk.candidates[0].content
                            and chunk.candidates[0].content.parts
                        ):
                            audio_bytes = (
                                chunk.candidates[0]
                                .content.parts[0]
                                .inline_data.data
                            )
                            if audio_bytes:
                                self.audio_player.write_stream_chunk(
                                    audio_bytes
                                )
                    except (IndexError, AttributeError, TypeError):
                        continue

                # Finalize the stream to stop the worker thread
                self.audio_player.finalize_stream()

            except Exception as e:
                self.get_logger().error(f"❌ Google Cloud TTS failed: {e}.")
                self.audio_player.stop()
        else:
            self.execute_local_piper(text_to_speak)


def main(args=None):
    rclpy.init(args=args)
    node = PiperTTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "audio_player"):
            node.audio_player.stop()
            node.audio_player.p.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()