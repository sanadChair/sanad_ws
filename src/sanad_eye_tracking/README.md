# 👁️ sanad_eye_tracking

**Real-time eye-tracking and gaze-based interaction for the SANAD smart wheelchair.**

This ROS 2 package uses a standard webcam to track a user's gaze and translate it into navigation commands and UI interactions — **no dedicated eye-tracker hardware required**. It ships two complementary components:

1. **ROS 2 Nodes** – integrate eye-tracking into the broader SANAD robot system.
2. **Standalone Pop-Up Display Demo** – a fully self-contained (ROS-free) demo showing the complete gaze-controlled wheelchair UI.

---

## Table of Contents

- [How It Works — High-Level Overview](#how-it-works--high-level-overview)
- [Core Algorithms](#core-algorithms)
  - [1. Face & Iris Detection (MediaPipe FaceLandmarker)](#1-face--iris-detection-mediapipe-facelandmarker)
  - [2. Gaze Projection (Iris Ratio Method)](#2-gaze-projection-iris-ratio-method)
  - [3. Eye Aspect Ratio (EAR) — Blink & Close Detection](#3-eye-aspect-ratio-ear--blink--close-detection)
  - [4. Exponential Moving Average (EMA) Smoothing](#4-exponential-moving-average-ema-smoothing)
  - [5. Blink-Rewind Buffer](#5-blink-rewind-buffer)
  - [6. Dwell-Click (Pop-Up Demo)](#6-dwell-click-pop-up-demo)
  - [7. Edge-Triggered Popup Visibility](#7-edge-triggered-popup-visibility)
- [Package Structure](#package-structure)
- [ROS 2 Nodes](#ros-2-nodes)
  - [eye_tracker_node](#eye_tracker_node)
  - [gaze_controller_node](#gaze_controller_node)
  - [ROS Topic Map](#ros-topic-map)
- [Tuneable Parameters (YAML)](#tuneable-parameters-yaml)
- [Standalone Pop-Up Display Demo](#standalone-pop-up-display-demo)
  - [Architecture](#architecture)
  - [Screens](#screens)
  - [Controls](#controls)
- [Installation & Dependencies](#installation--dependencies)
- [Building & Running (ROS 2)](#building--running-ros-2)
- [Running the Standalone Demo](#running-the-standalone-demo)
- [Monitoring ROS Topics](#monitoring-ros-topics)

---

## How It Works — High-Level Overview

```
┌─────────────┐     BGR frames     ┌──────────────────────────┐
│   Webcam    │ ─────────────────▶ │   eye_tracker_node.py    │
└─────────────┘                    │  (MediaPipe FaceLandmark) │
                                   │  • 478 landmark detection │
                                   │  • Iris centre extraction │
                                   │  • EAR blink detection    │
                                   │  • EMA smoothing          │
                                   └──────────┬───────────────┘
                                              │ /eye_tracking/gaze_position  "x,y"
                                              │ /eye_tracking/eye_state      "open|blink|closed"
                                              ▼
                                   ┌──────────────────────────┐
                                   │  gaze_controller_node.py │
                                   │  • Cursor freeze on blink │
                                   │  • 2-s close → click cmd │
                                   │  • Command cooldown       │
                                   └──────────┬───────────────┘
                                              │ /eye_tracking/command         "click"
                                              │ /eye_tracking/cursor_position "x,y"
                                              ▼
                                        Your ROS 2 App
```

> [!NOTE]
> The **standalone demo** (`popup_display_demo/`) contains the same gaze engine logic but runs entirely without ROS, as a pure Python + OpenCV application.

---

## Core Algorithms

### 1. Face & Iris Detection (MediaPipe FaceLandmarker)

The package uses **Google's MediaPipe FaceLandmarker** (Tasks API, `mediapipe >= 0.10`), which outputs **478 facial landmarks** per detected face. The model file (`face_landmarker.task`) is a float16-quantised neural network that runs in real-time on CPU.

| Landmark Group | Indices | Used For |
|---|---|---|
| Left iris (user's right eye) | 469–472 | Gaze X/Y |
| Right iris (user's left eye) | 474–477 | Gaze X/Y |
| Left eye corners | 33 (inner), 133 (outer) | Gaze ratio normalisation |
| Right eye corners | 362 (inner), 263 (outer) | Gaze ratio normalisation |
| Left eye top/bottom | 159 / 145 | EAR blink detection |
| Right eye top/bottom | 386 / 374 | EAR blink detection |

The model is run in **VIDEO mode** (`RunningMode.VIDEO`), meaning each frame is fed with a monotonically increasing timestamp (in ms), allowing the model to leverage temporal context for improved tracking stability.

---

### 2. Gaze Projection (Iris Ratio Method)

This is the main algorithm that converts raw landmark coordinates into a normalized screen position `(gaze_x, gaze_y)` where `0.0 = left/top` and `1.0 = right/bottom`.

**Step 1 – Compute iris centre**

The four iris landmarks per eye are averaged to find the iris centre pixel:

```
iris_x = mean(landmark[i].x * frame_width   for i in iris_indices)
iris_y = mean(landmark[i].y * frame_height  for i in iris_indices)
```

**Step 2 – Compute horizontal and vertical ratios**

The iris position is expressed as a ratio *within the eye socket*, making it **head-pose-independent** (small head movements don't affect the gaze reading):

```
horizontal_ratio = (iris_x - eye_left_corner_x) / eye_width  -  0.5
# Result: 0.0 = looking centre, negative = looking left, positive = looking right

vertical_ratio = (iris_y - eye_midline_y) / eye_width
# Normalised by eye WIDTH (not height) for consistency
```

Both eyes are computed independently and their ratios are **averaged** to reduce noise from one eye being partially occluded.

**Step 3 – Apply offset and range normalization**

The raw ratio is mapped to `[-1, 1]` using configurable offset and range parameters (which compensate for natural resting-gaze bias and camera-specific iris motion range):

```
norm_h = (h_ratio - h_offset) / h_range
norm_v = (v_ratio - v_offset) / v_range
```

**Step 4 – Non-linear power curve**

A mild power curve `x → sign(x) * |x|^1.2` is applied to each axis. This makes small, precise centre movements smoother while still allowing edge extremes to be reached:

```
norm_h = sign(norm_h) * |norm_h| ^ 1.2
norm_v = sign(norm_v) * |norm_v| ^ 1.2
```

**Step 5 – Clamp and remap to `[0, 1]`**

```
norm_h = clamp(norm_h, -1.0, 1.0)
gaze_x = (norm_h + 1.0) / 2.0   # 0.0 = left,  1.0 = right
gaze_y = (norm_v + 1.0) / 2.0   # 0.0 = top,   1.0 = bottom
```

---

### 3. Eye Aspect Ratio (EAR) — Blink & Close Detection

The **Eye Aspect Ratio** is a well-established metric from Soukupová & Čech (2016) for detecting blinks and prolonged eye closure from geometric facial landmarks:

```
EAR = vertical_distance / horizontal_distance
    = ||top - bottom|| / ||left_corner - right_corner||
```

- **High EAR (≥ threshold)**: Eyes are open. Gaze tracking is active.
- **Low EAR (< threshold)**: Eyes are closed.
  - If the *previous* state was `open` → emit `"blink"` (transition event).
  - If the previous state was already closed → emit `"closed"` (sustained closure).

The threshold default is `ear_threshold = 0.20`. Values typically range from ~0.30 (wide open) to ~0.05 (fully closed); the threshold sits between these extremes.

---

### 4. Exponential Moving Average (EMA) Smoothing

Raw iris positions are inherently jittery. An Exponential Moving Average is applied **only when eyes are open** (closed/blink states freeze the cursor — see below):

```
smoothed_x = α × raw_x  +  (1 - α) × smoothed_x
smoothed_y = α × raw_y  +  (1 - α) × smoothed_y
```

Where `α = gaze_smoothing` (default `0.3`).

| `α` value | Behaviour |
|---|---|
| Low (e.g. 0.1) | Very smooth, but noticeably laggy / slow to respond |
| Medium (0.3)   | Balanced — the default |
| High (e.g. 0.7) | Fast response, but jittery |

---

### 5. Blink-Rewind Buffer

When a user blinks, their eyelids physically push the iris *downward* before closing. Without correction, this causes the gaze cursor to jump to the bottom of the screen every time the user blinks — a frustrating interaction artefact.

**Solution:** A **circular history buffer** (`deque(maxlen=7)`, covering ~230 ms of history) stores the last 7 open-eye gaze positions. When a blink is detected, the cursor position is *rewound* to the oldest position in the buffer (i.e., the state *before* the eyelid started moving):

```python
if avg_ear < ear_threshold:
    if len(gaze_history) > 0:
        smoothed_x, smoothed_y = gaze_history[0]  # rewind
```

This makes blinks transparent to the cursor position.

---

### 6. Dwell-Click (Pop-Up Demo)

In the standalone demo, the user activates buttons **hands-free** using a gaze dwell-click:

1. Looking near a screen edge **reveals** the popup buttons (see next section).
2. When the gaze lands **inside** a button's bounding box (with a generous 12+ px padding for easier targeting), a timer starts.
3. If the gaze remains on the button for **`DWELL_DURATION` seconds** (default 3.0 s in `popup_ui.py`, displayed in the Settings screen as "1.5 s"), the button activates.
4. A visual **progress arc** fills clockwise around a small circle on the button during the dwell, giving the user clear feedback.
5. A **cooldown** (`COOLDOWN = 0.6 s`) prevents accidental double-activations.

In the **ROS nodes**, the equivalent mechanism is:
- Eyes must stay `"closed"` continuously for `click_hold_time = 2.0 s` → emits a `"click"` command on `/eye_tracking/command`.

---

### 7. Edge-Triggered Popup Visibility

Buttons are hidden when the user is looking at the main content area. They smoothly **fade in** only when the gaze approaches a screen edge:

```
show_button = (gaze_x < EDGE_TRIGGER)         # left edge
           OR (gaze_x > 1.0 - EDGE_TRIGGER)   # right edge
           OR (gaze_y < EDGE_TRIGGER)          # top edge
           OR (gaze_y > 1.0 - EDGE_TRIGGER)   # bottom edge

EDGE_TRIGGER = 0.18  (18% of the screen from each edge)
```

Opacity transitions smoothly using a simple lerp: `opacity += ±FADE_SPEED (0.12)` per frame.

---

## Package Structure

```
sanad_eye_tracking/
│
├── sanad_eye_tracking/               # ROS 2 Python package (the actual nodes)
│   ├── __init__.py
│   ├── eye_tracker_node.py           # Node 1: Camera + MediaPipe + gaze projection
│   └── gaze_controller_node.py       # Node 2: Command state machine
│
├── popup_display_demo/               # Standalone (no-ROS) demo application
│   ├── main.py                       # Entry point — wires everything together
│   ├── gaze_engine.py                # ROS-free version of the gaze tracker
│   ├── popup_ui.py                   # 8-button edge-popup dwell-click system
│   ├── screens.py                    # 5 screen definitions (Home, Nav, Comm, Health, Settings)
│   └── README.md                     # Demo-specific quick-start guide
│
├── config/
│   ├── eye_tracking_params.yaml      # All tuneable parameters for ROS nodes
│   └── face_landmarker.task          # Bundled MediaPipe model (float16, ~3.6 MB)
│
├── launch/
│   └── eye_tracking.launch.py        # Launches both ROS nodes with the YAML config
│
├── resource/sanad_eye_tracking       # ROS ament resource marker
├── test/                             # Linting & style tests
├── package.xml                       # ROS 2 package manifest
├── setup.cfg
└── setup.py
```

---

## ROS 2 Nodes

### eye_tracker_node

**File:** `sanad_eye_tracking/eye_tracker_node.py`
**Executable:** `eye_tracker`

This is the **perception node**. It opens the webcam, runs the MediaPipe FaceLandmarker on every frame, and publishes continuous gaze data.

| Publisher Topic | Message Type | Description |
|---|---|---|
| `/eye_tracking/gaze_position` | `std_msgs/String` | `"x,y"` — floats in `[0.0, 1.0]`. `0.0,0.0` = top-left corner. |
| `/eye_tracking/eye_state` | `std_msgs/String` | `"open"`, `"blink"`, or `"closed"` |

| Subscriber Topic | Message Type | Description |
|---|---|---|
| `/eye_tracking/command` | `std_msgs/String` | Listens for `"click"` to record last-click position in debug overlay |

**What it does per timer tick (at `publish_rate` Hz):**
1. Reads one frame from the webcam and horizontally mirrors it (so the user's natural left/right is preserved).
2. Converts the frame to RGB and passes it to `FaceLandmarker.detect_for_video()`.
3. If a face is detected, extracts iris centres and eye corners, computes the gaze ratio, applies the offset/range/power-curve mapping, and runs EMA smoothing.
4. Computes EAR; if below threshold, triggers blink-rewind and sets eye state.
5. Publishes `gaze_position` and `eye_state`.
6. If `show_debug_window=true`, overlays iris circles, crosshair cursor, last-click marker, and live numerical readouts on the frame.

---

### gaze_controller_node

**File:** `sanad_eye_tracking/gaze_controller_node.py`
**Executable:** `gaze_controller`

This is the **logic node**. It subscribes to the raw perception output and implements the interaction state machine.

| Publisher Topic | Message Type | Description |
|---|---|---|
| `/eye_tracking/command` | `std_msgs/String` | `"click"` when eyes held closed ≥ `click_hold_time` |
| `/eye_tracking/cursor_position` | `std_msgs/String` | `"x,y"` — current frozen or live cursor position |

| Subscriber Topic | Message Type | Description |
|---|---|---|
| `/eye_tracking/gaze_position` | `std_msgs/String` | Raw gaze from `eye_tracker_node` |
| `/eye_tracking/eye_state` | `std_msgs/String` | `"open"` / `"blink"` / `"closed"` from `eye_tracker_node` |

**State machine logic:**

```
eye_state == "open"   →  cursor tracks gaze_position
eye_state == "blink"  →  cursor FREEZES, click timer resets
eye_state == "closed" →  cursor FREEZES
                          if closed for >= click_hold_time seconds:
                              publish "click" on /eye_tracking/command
                              start command_cooldown
```

The cursor position is **always published** (even when frozen), so downstream subscribers always have an up-to-date value. A cooldown between commands prevents repeated clicks from a single prolonged closure.

---

### ROS Topic Map

```
eye_tracker_node ──────────────────────────────────────▶ gaze_controller_node
  /eye_tracking/gaze_position  ──────────────────────▶  subscribes
  /eye_tracking/eye_state      ──────────────────────▶  subscribes

gaze_controller_node ──────────────────────────────────▶ Your App
  /eye_tracking/command        ──────────────────────▶  subscribes
  /eye_tracking/cursor_position ─────────────────────▶  subscribes

Your App ──────────────────────────────────────────────▶ eye_tracker_node (optional)
  /eye_tracking/command        ──────────────────────▶  subscribes (for debug overlay)
```

---

## Tuneable Parameters (YAML)

All parameters are defined in `config/eye_tracking_params.yaml` and are loaded at launch. **No recompile is required** to change them.

### `eye_tracker` node parameters

| Parameter | Default | Description |
|---|---|---|
| `camera_index` | `0` | Webcam device index. Use `1`, `2`, etc. for external cameras. |
| `publish_rate` | `30.0` | Publishing frequency in Hz. Lower values reduce CPU load. |
| `gaze_range_horizontal` | `0.040` | Maximum expected horizontal iris ratio (eye-relative units). Controls how much of the screen the full left-to-right eye movement covers. **Decrease** to make small eye movements cover a larger screen area. |
| `gaze_range_vertical` | `0.020` | Same as above for the vertical axis. Typically smaller because eyes move less vertically. |
| `gaze_offset_horizontal` | `0.000` | Calibration offset added to the raw horizontal ratio before normalization. Adjust if the cursor rests left or right of centre when looking straight ahead. |
| `gaze_offset_vertical` | `-0.015` | Calibration offset for the vertical axis. The default `-0.015` corrects for the iris typically sitting slightly below the eye midline. |
| `gaze_smoothing` | `0.3` | EMA factor `α`. Range `[0.0, 1.0]`. Lower = smoother but laggier. |
| `ear_threshold` | `0.20` | EAR value below which eyes are considered "closed". Increase if blinks aren't detected; decrease if normal open eyes trigger false blinks. |
| `show_debug_window` | `true` | If `true`, opens an OpenCV window showing the camera feed with gaze overlays. Set to `false` in deployment. |
| `model_path` | `""` | Explicit path to `face_landmarker.task`. If empty, the node looks in the installed `share/config/` directory automatically. |

### `gaze_controller` node parameters

| Parameter | Default | Description |
|---|---|---|
| `click_hold_time` | `2.0` | Seconds of continuous eye closure required to trigger a `"click"` command. Increase if accidental clicks occur; decrease for faster interaction. |
| `command_cooldown` | `1.0` | Minimum seconds between any two emitted commands. Prevents rapid repeated clicks. |

> [!TIP]
> **Calibration workflow:**
> 1. Run with `show_debug_window: true`.
> 2. Look at the centre of the screen — if the crosshair cursor is not centred, adjust `gaze_offset_horizontal` and `gaze_offset_vertical`.
> 3. Look as far left as you can — if the cursor doesn't reach the left edge, decrease `gaze_range_horizontal`.
> 4. Blink naturally — if `EAR` in the debug overlay doesn't drop below `ear_threshold`, increase the threshold.

---

## Standalone Pop-Up Display Demo

**Location:** `popup_display_demo/`

This is a **complete, ROS-independent** demonstration of the eye-tracking UI. It was built to test and showcase the interaction model without requiring a full ROS 2 environment. It runs as a fullscreen OpenCV application.

### Architecture

```
main.py (SANADApp)
 ├── GazeEngine (gaze_engine.py)
 │    └── Same iris-ratio + EAR + EMA pipeline as the ROS node
 │        Outputs: (gaze_x, gaze_y, eye_state, camera_frame)
 │
 ├── PopupSystem (popup_ui.py)
 │    ├── 8 × PopupButton objects (top-left, top-center, top-right,
 │    │                            mid-left,              mid-right,
 │    │                            bottom-left, bottom-center, bottom-right)
 │    ├── Edge-trigger visibility logic
 │    └── Dwell-click state machine per button
 │
 └── 5 × Screen objects (screens.py)
      ├── HomeScreen
      ├── NavigationScreen
      ├── CommunicationScreen
      ├── HealthScreen
      └── SettingsScreen
```

**Main loop (per frame):**
1. `GazeEngine.process_frame()` → `(gaze_x, gaze_y, eye_state, cam_frame)`
2. Current screen's `draw(canvas)` renders the UI.
3. `PopupSystem.update(gaze_x, gaze_y, ...)` updates button visibility + dwell, returns `action_id` if a button fires.
4. `_handle_action(action_id)` either switches the screen or shows a notification banner.
5. `_draw_cursor(canvas, gaze_x, gaze_y, eye_state)` overlays the gaze cursor with a fading trail.
6. A small camera thumbnail is drawn in the bottom-right corner for debugging.
7. `cv2.imshow()` displays the frame.

### Screens

Each screen implements `draw(canvas)` and `get_popup_buttons()` (returns 8 `(label, action_id)` tuples).

| Screen | Key | Accent Colour | Content | Popup Actions |
|---|---|---|---|---|
| **Home** | `h` | Orange | Dashboard with Navigation, Communication, Health, Settings cards + battery/speed/connection status | Navigate, Speed Up, Communicate, Emergency, Call Help, Health, Stop, Settings |
| **Navigation** | `n` | Blue | Animated route map with grid overlay, animated position marker, turn-by-turn card, distance/obstacle/terrain cards | Go Home, Speed Up, Turn Right, Turn Left, Reverse, Stop, Reroute, Emergency |
| **Communication** | `c` | Green | 8 quick-phrase cards (Yes, No, Need water, Need help, etc.) + recent message log | Go Home, Say Yes, Say No, Need Water, Need Help, Feel Pain, Call Family, Emergency |
| **Health Monitor** | `m` | Red | Animated ECG waveform with live BPM simulation, SpO2, temperature, blood pressure, stress level, daily activity progress bar | Go Home, Call Doctor, Alert Nurse, Log Symptom, Take Meds, Pain Level, View History, Emergency |
| **Settings** | `s` | Purple | 8 setting rows (Speed Limit, Volume, Brightness, Gaze Sensitivity, Dwell Time, Obstacle Avoidance, Auto-Brake, WiFi) with progress bar indicators | Go Home, Speed +/-, Volume +/-, Brightness +/-, Reset All |

All screens share a common **header bar** (accent-coloured bottom line), **status bar** (battery, speed, mode, clock), and the Ubuntu **Yaru Dark** colour palette.

### Controls

| Method | Action |
|---|---|
| **Look toward screen edge** | Reveals popup buttons (fade-in) |
| **Dwell on a button (3 s)** | Activates it (progress arc shows countdown) |
| Keyboard `h` | Switch to Home screen |
| Keyboard `n` | Switch to Navigation screen |
| Keyboard `c` | Switch to Communication screen |
| Keyboard `m` | Switch to Health Monitor screen |
| Keyboard `s` | Switch to Settings screen |
| Keyboard `q` or `ESC` | Quit the application |

---

## Installation & Dependencies

### Python packages

```bash
pip install mediapipe opencv-python numpy
```

| Package | Version Required | Purpose |
|---|---|---|
| `mediapipe` | `>= 0.10` | FaceLandmarker Tasks API (iris landmarks) |
| `opencv-python` | Any recent | Camera capture, rendering |
| `numpy` | Any recent | Vectorised landmark math |

### MediaPipe model file

The `face_landmarker.task` model (~3.6 MB) is bundled in `config/`. If it is missing, download it with:

```bash
wget -O config/face_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/latest/face_landmarker.task
```

### ROS 2 dependencies (for the nodes only)

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- ROS 2 Humble or newer recommended

---

## Building & Running (ROS 2)

```bash
# 1. Build the package
cd ~/sanad_ws         # or wherever your workspace lives
colcon build --packages-select sanad_eye_tracking
source install/setup.bash

# 2. Launch both nodes
ros2 launch sanad_eye_tracking eye_tracking.launch.py
```

The launch file loads `config/eye_tracking_params.yaml` and starts:
- `eye_tracker` — the perception node
- `gaze_controller` — the command state machine

---

## Running the Standalone Demo

```bash
cd sanad_ws/src/sanad_eye_tracking/popup_display_demo

# Default (camera 0, auto model path)
python3 main.py

# Custom camera or model
python3 main.py --camera 1 --model /path/to/face_landmarker.task
```

The demo opens in **fullscreen** at `1920×1080`. It will detect your screen resolution automatically via `xrandr` or `xdpyinfo` and fall back to `1920×1080` if detection fails.

---

## Monitoring ROS Topics

```bash
# Live gaze position (normalised 0.0-1.0)
ros2 topic echo /eye_tracking/gaze_position

# Eye state (open / blink / closed)
ros2 topic echo /eye_tracking/eye_state

# Current cursor position (frozen on blink)
ros2 topic echo /eye_tracking/cursor_position

# Commands emitted (click)
ros2 topic echo /eye_tracking/command

# Topic publish rates
ros2 topic hz /eye_tracking/gaze_position
```

---

> [!IMPORTANT]
> **Lighting matters.** MediaPipe FaceLandmarker performs best in even, frontal lighting. Avoid strong backlight (e.g. sitting in front of a window). Keep your face roughly centred in frame and within ~60–80 cm of the camera.

> [!TIP]
> If the cursor consistently drifts in one direction even when looking straight ahead, adjust `gaze_offset_horizontal` or `gaze_offset_vertical` in `eye_tracking_params.yaml` (for ROS) or the `GazeEngine` constructor arguments (for the standalone demo).
