#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import threading
import queue
import numpy as np
import sounddevice as sd
import soundfile as sf


class AudioPlayer:
    def __init__(self, node):
        self.node = node
        self.is_playing = False
        self.stop_event = threading.Event()
        self.audio_queue = None
        self.thread = None
        self._stream = None

    # ------------------------------------------------------------------ #
    #  Stop everything                                                     #
    # ------------------------------------------------------------------ #
    def stop(self):
        self.stop_event.set()
        self.is_playing = False
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None
        # drain queue so the worker thread unblocks
        if self.audio_queue is not None:
            try:
                while True:
                    self.audio_queue.get_nowait()
            except queue.Empty:
                pass
            self.audio_queue.put(None)   # wake worker if it's blocking

    # ------------------------------------------------------------------ #
    #  WAV playback (local Piper TTS)                                      #
    # ------------------------------------------------------------------ #
    def play_wav(self, file_path):
        self.stop()
        self.stop_event.clear()
        self.is_playing = True

        def run():
            try:
                data, fs = sf.read(file_path, dtype="int16")
                sd.play(data, samplerate=fs)
                while sd.get_stream().active:
                    if self.stop_event.is_set():
                        sd.stop()
                        break
                    threading.Event().wait(0.05)
            except Exception as e:
                self.node.get_logger().error(f"Failed to play WAV {file_path}: {e}")
            finally:
                self.is_playing = False

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ #
    #  Streaming playback via continuous OutputStream (Gemini cloud TTS)  #
    # ------------------------------------------------------------------ #
    def play_stream(self, rate=24000, channels=1, output_rate=44100):
        self.stop()
        self.stop_event.clear()
        self.input_rate = rate
        self.output_rate = output_rate
        self.is_playing = True
        self.audio_queue = queue.Queue()

        def run():
            try:
                with sd.OutputStream(
                    samplerate=self.output_rate,
                    channels=channels,
                    dtype="int16",
                ) as stream:
                    self._stream = stream
                    self.node.get_logger().info(
                        f"🔊 Audio stream opened: {self.output_rate} Hz"
                    )
                    while not self.stop_event.is_set():
                        try:
                            chunk = self.audio_queue.get(timeout=0.1)
                            if chunk is None:       # sentinel → clean finish
                                break
                            stream.write(chunk)
                        except queue.Empty:
                            continue
                        except Exception as e:
                            self.node.get_logger().error(
                                f"Error writing to OutputStream: {e}"
                            )
                            break
            except Exception as e:
                self.node.get_logger().error(
                    f"Failed to open sounddevice OutputStream: {e}"
                )
            finally:
                self._stream = None
                self.is_playing = False

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ #
    #  Enqueue a raw-PCM bytes chunk (resampled before queuing)           #
    # ------------------------------------------------------------------ #
    def write_stream_chunk(self, chunk: bytes):
        if not self.is_playing or self.audio_queue is None:
            return
        try:
            audio_np = np.frombuffer(chunk, dtype=np.int16).astype(np.float32)
            if len(audio_np) == 0:
                return

            # Resample input_rate → output_rate via linear interpolation
            num_out = int(len(audio_np) * self.output_rate / self.input_rate)
            if num_out == 0:
                return
            indices = np.linspace(0, len(audio_np) - 1, num_out)
            resampled = np.interp(
                indices, np.arange(len(audio_np)), audio_np
            ).astype(np.int16)

            # sd.OutputStream.write() needs shape (frames, channels)
            self.audio_queue.put(resampled.reshape(-1, 1))
        except Exception as e:
            self.node.get_logger().error(f"Error resampling chunk: {e}")

    def finalize_stream(self):
        """Signal the playback thread that the stream is done."""
        if self.audio_queue is not None:
            self.audio_queue.put(None)


# --------------------------------------------------------------------------- #


class PiperTTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")

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
        self.audio_player = AudioPlayer(self)

        self.get_logger().info(
            "Arabic TTS Node Ready (sounddevice OutputStream). "
            "Listening on /robot/speak ..."
        )

    # ------------------------------------------------------------------ #
    #  Local Piper TTS                                                     #
    # ------------------------------------------------------------------ #
    def execute_local_piper(self, text_to_speak: str):
        try:
            subprocess.run(
                ["piper", "--model", self.model_path,
                 "--output_file", self.temp_audio_file],
                input=text_to_speak.encode("utf-8"),
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self.audio_player.play_wav(self.temp_audio_file)
        except subprocess.CalledProcessError as e:
            err = e.stderr.decode("utf-8") if e.stderr else "Unknown Piper error"
            self.get_logger().error(f"Piper crashed: {err}")

    # ------------------------------------------------------------------ #
    #  Main callback                                                       #
    # ------------------------------------------------------------------ #
    def speak_callback(self, msg: String):
        text_to_speak = msg.data

        self.audio_player.stop()
        self.get_logger().info(f"Speaking: {text_to_speak}")

        use_cloud = self.get_parameter("use_cloud").get_parameter_value().bool_value

        if use_cloud:
            try:
                from google import genai
                from google.genai import types

                self.get_logger().info("Connecting to Google Cloud Vertex AI TTS ...")
                client = genai.Client(
                    vertexai=True,
                    project="polar-ray-459012-b0",
                    location="us-central1",
                )

                # Open the OutputStream BEFORE the first chunk arrives
                self.audio_player.play_stream(
                    rate=24000, channels=1, output_rate=44100
                )

                response_stream = client.models.generate_content_stream(
                    model="gemini-2.5-flash-preview-tts",
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

                chunk_count = 0
                for chunk in response_stream:
                    if not self.audio_player.is_playing:
                        break
                    try:
                        if (
                            chunk.candidates
                            and chunk.candidates[0].content
                            and chunk.candidates[0].content.parts
                        ):
                            audio_bytes = (
                                chunk.candidates[0].content.parts[0].inline_data.data
                            )
                            if audio_bytes:
                                chunk_count += 1
                                self.audio_player.write_stream_chunk(audio_bytes)
                    except (IndexError, AttributeError, TypeError):
                        continue

                self.get_logger().info(f"Stream done - {chunk_count} chunks received")
                self.audio_player.finalize_stream()

            except Exception as e:
                self.get_logger().error(f"Google Cloud TTS failed: {e}")
                self.audio_player.stop()
        else:
            self.execute_local_piper(text_to_speak)


# --------------------------------------------------------------------------- #


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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()