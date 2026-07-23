# Whisper.cpp Setup Guide (Local Speech-to-Text)

This guide covers installing and configuring [whisper.cpp](https://github.com/ggerganov/whisper.cpp) for offline, local speech-to-text (STT) in Sanad.

Use this when running in **local mode** (`use_cloud:=false`), where speech recognition is handled on-device without any internet connection.

---

## Prerequisites

- Git
- CMake 3.13+
- A C++17-capable compiler (GCC 9+ or Clang 10+)
- ~2–5 GB of disk space for the model file

---

## 1. Clone and Build whisper.cpp

```bash
cd ~
git clone https://github.com/ggerganov/whisper.cpp.git
cd whisper.cpp
cmake -B build && cmake --build build --config Release
```

---

## 2. Download the Speech Model

Sanad uses the `large-v3-q5_0` quantised model for a good balance of accuracy and speed:

```bash
bash ./models/download-ggml-model.sh large-v3-q5_0
```

> **Note:** The model file is approximately 1.1 GB. Make sure you have a stable internet connection for this step.

If you prefer a lighter / faster model, choose one of the following instead:

| Model            | Size   | Notes                          |
| ---------------- | ------ | ------------------------------ |
| `base`           | ~150 MB | Fast, lower accuracy           |
| `small`          | ~480 MB | Good balance for English       |
| `medium`         | ~1.5 GB | Better multilingual accuracy   |
| `large-v3-q5_0`  | ~1.1 GB | **Recommended** — Arabic + EN  |

---

## 3. Verify the Build

Run a quick test to confirm whisper.cpp is working:

```bash
cd ~/whisper.cpp
./build/bin/whisper-cli -m models/ggml-large-v3-q5_0.bin -f samples/jfk.wav
```

You should see a transcript of the audio sample printed to the terminal.

---

## 4. Configure Sanad to Use Local STT

Launch the voice stack with `use_cloud:=false`:

```bash
ros2 launch sanad_voice_control commander.launch.py use_cloud:=false
```

---

## Troubleshooting

- **`cmake: command not found`:** Install CMake via `sudo apt install cmake`.
- **Build fails with CUDA errors:** whisper.cpp supports GPU acceleration but it is optional. Build without it by default; CMake will automatically use CPU only.
- **Model file not found:** Ensure the model was downloaded to `~/whisper.cpp/models/`. The filename must match what `whisper_node` expects (configurable as a ROS 2 parameter).
- **Poor Arabic recognition:** The `large-v3-q5_0` model performs best for Arabic. Smaller models may struggle with non-English speech.
