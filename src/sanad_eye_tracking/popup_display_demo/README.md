# SANAD Pop-Up Display Demo

Eye-controlled pop-up interface for the SANAD smart wheelchair.
**No ROS dependency** – runs as a standalone Python application.

## Quick Start

```bash
cd popup_display_demo
python3 main.py
```

### Options
```
--camera N      Camera device index (default: 0)
--model PATH    Path to face_landmarker.task model
```

If `--model` is not provided, the app looks for
`../config/face_landmarker.task` relative to this folder.

## How It Works

1. **Gaze Engine** (`gaze_engine.py`) – extracts the eye projection logic
   from `eye_tracker_node.py` without any ROS dependency. Produces
   normalised `(gaze_x, gaze_y)` in `[0, 1]` and an `eye_state`.

2. **Pop-Up UI** (`popup_ui.py`) – 8 buttons positioned around the window
   edges (top-left/center/right, mid-left/right, bottom-left/center/right).
   Buttons are **hidden** until your gaze approaches the corresponding
   edge region, then **fade in**. A **1.5-second dwell** (continuous
   gaze on a button) triggers a click, shown via an animated arc.

3. **Screens** (`screens.py`) – 5 themed screens for a smart wheelchair:
   - **Home** – dashboard with quick-status cards
   - **Navigation** – route map with animated path, turn instructions
   - **Communication** – quick phrase board for non-verbal users
   - **Health Monitor** – live vitals with animated ECG
   - **Settings** – system preferences with progress bars

4. **Main** (`main.py`) – wires everything together in an OpenCV loop.

## Keyboard Fallback

| Key | Action |
|-----|--------|
| `h` | Home screen |
| `n` | Navigation screen |
| `c` | Communication screen |
| `m` | Health monitor screen |
| `s` | Settings screen |
| `q` / `ESC` | Quit |

## Architecture

```
popup_display_demo/
├── __init__.py
├── main.py          # Entry point & app loop
├── gaze_engine.py   # Standalone eye tracker
├── popup_ui.py      # 8-button popup system
├── screens.py       # Screen definitions
└── README.md
```
