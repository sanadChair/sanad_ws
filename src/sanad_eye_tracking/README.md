# 👁️ sanad_eye_tracking — Package Overview

## What Was Created

A new ROS2 Python package **`sanad_eye_tracking`** under `sanad_ws/src/` with two nodes that work together:

```mermaid
graph LR
    CAM[🎥 Webcam] --> ET[eye_tracker node]
    ET -- /eye_tracking/gaze_direction --> GC[gaze_controller node]
    ET -- /eye_tracking/eye_state --> GC
    GC -- /eye_tracking/command --> APP[Your App]
    GC -- /eye_tracking/grid_position --> APP
```

---

## Nodes

### 1. `eye_tracker` — Camera + MediaPipe Face Mesh

| Topic Published | Type | Values |
|---|---|---|
| `/eye_tracking/gaze_direction` | `std_msgs/String` | `north`, `south`, `east`, `west`, `center` |
| `/eye_tracking/eye_state` | `std_msgs/String` | `open`, `blink`, `closed` |

**How it works:**
- Captures frames from the webcam
- Uses **MediaPipe Face Mesh** with iris refinement (landmarks 468–477)
- Computes normalised iris offset relative to eye corners → gaze direction
- Computes **Eye Aspect Ratio (EAR)** → blink / closed detection
- Optional debug window shows landmarks + live status

### 2. `gaze_controller` — Timing & Command State Machine

| Topic Published | Type | Values |
|---|---|---|
| `/eye_tracking/command` | `std_msgs/String` | `move_north`, `move_south`, `move_east`, `move_west`, `click`, `go_back` |
| `/eye_tracking/grid_position` | `std_msgs/String` | `row,col` (e.g. `1,1`) |

**Control mapping:**

| User Action | Duration/Pattern | Command Emitted |
|---|---|---|
| Look North | 3 s continuous | `move_north` — cursor moves up one row |
| Look South | 3 s continuous | `move_south` — cursor moves down one row |
| Look East | 3 s continuous | `move_east` — cursor moves right one col |
| Look West | 3 s continuous | `move_west` — cursor moves left one col |
| Look Center | 3 s continuous | `click` — select current window |
| Close Eyes | 3 s continuous | `go_back` — navigate back |

---

## File Structure

```
sanad_eye_tracking/
├── config/
│   └── eye_tracking_params.yaml      # All tuneable thresholds
├── launch/
│   └── eye_tracking.launch.py        # Launches both nodes
├── sanad_eye_tracking/
│   ├── __init__.py
│   ├── eye_tracker_node.py           # Camera + MediaPipe detection
│   └── gaze_controller_node.py       # Timing logic + grid cursor
├── resource/sanad_eye_tracking
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

---

## How to Build & Run

### Prerequisites
```bash
pip install mediapipe opencv-python numpy
```

### Build
```bash
cd ~/sanad_ws
colcon build --packages-select sanad_eye_tracking
source install/setup.bash
```

### Run
```bash
ros2 launch sanad_eye_tracking eye_tracking.launch.py
```

### Monitor output
```bash
# Watch commands being emitted
ros2 topic echo /eye_tracking/command

# Watch grid position
ros2 topic echo /eye_tracking/grid_position

# Watch raw gaze
ros2 topic echo /eye_tracking/gaze_direction
```

---

## Tuneable Parameters

All parameters live in [eye_tracking_params.yaml](file:///media/mazen/Windows/Projects/ROS/GraduationProject26/sanad_ws/src/sanad_eye_tracking/config/eye_tracking_params.yaml) and can be adjusted without recompiling:

| Parameter | Default | Description |
|---|---|---|
| `camera_index` | `0` | Webcam device index |
| `gaze_horizontal_threshold` | `0.015` | How far iris must shift horizontally |
| `gaze_vertical_threshold` | `0.012` | How far iris must shift vertically |
| `ear_threshold` | `0.20` | EAR below this = eyes closed |
| `direction_hold_time` | `3.0` s | Time gaze must hold to trigger move |
| `close_hold_time` | `3.0` s | Time eyes must stay closed for go_back |
| `double_blink_max_interval` | `0.6` s | Max gap between blinks for double-blink |
| `grid_rows` / `grid_cols` | `3 × 3` | Grid layout for window navigation |

> [!TIP]
> If the tracker is too sensitive or not sensitive enough, start by adjusting `gaze_horizontal_threshold` and `gaze_vertical_threshold`. Lower = more sensitive.

> [!IMPORTANT]
> Make sure your webcam is unobstructed and you have good lighting. MediaPipe performs best with frontal face orientation.
