#!/usr/bin/env python3
"""
main.py – SANAD Pop-Up Display Demo

Standalone demo (no ROS) combining:
  • GazeEngine  – real-time eye tracking / gaze projection
  • PopupSystem – 8 edge-triggered dwell-click buttons
  • 5 Screens   – Home, Navigation, Communication, Health, Settings

Usage:
    python3 main.py [--camera 0] [--model path/to/face_landmarker.task]

Controls:
    • Move your eyes to the screen edges to reveal popup buttons
    • Dwell (look at a button for 1.5 s) to activate it
    • Press 'q' or ESC to quit
    • Press 'h' to go home (keyboard fallback)
"""

import argparse
import sys
import os
import cv2
import numpy as np
import time

# Ensure this package is importable
sys.path.insert(0, os.path.dirname(__file__))

from gaze_engine import GazeEngine
from popup_ui import PopupSystem
from screens import ALL_SCREENS, C


# ═══════════════════════════════════════════════════════════════════════
#  Application
# ═══════════════════════════════════════════════════════════════════════
class SANADApp:
    WINDOW_NAME = "SANAD Eye-Controlled Display"

    def __init__(self, camera_index=0, model_path=None):
        # ── Set screen resolution ────────────────────────────────
        self.WIN_W, self.WIN_H = 1920, 1080
        print(f"[SANAD] Screen resolution fixed to: {self.WIN_W}x{self.WIN_H}")

        # ── Gaze engine ─────────────────────────────────────────────
        self.gaze = GazeEngine(camera_index=camera_index, model_path=model_path)
        print("[SANAD] Gaze engine initialised  ✓")

        # ── Pop-up system ───────────────────────────────────────────
        self.popup = PopupSystem()

        # ── Screens ─────────────────────────────────────────────────
        self.screens = {name: cls() for name, cls in ALL_SCREENS.items()}
        self.current_screen_name = "home"
        self._load_screen("home")

        # ── UI canvas ───────────────────────────────────────────────
        self.canvas = np.zeros((self.WIN_H, self.WIN_W, 3), dtype=np.uint8)

        # ── Fullscreen window ───────────────────────────────────────
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                              cv2.WINDOW_FULLSCREEN)

        # ── Notification system ─────────────────────────────────────
        self._notification = None     # (message, expire_time, colour)

        # ── Cursor trail ────────────────────────────────────────────
        self._cursor_trail = []

        print("[SANAD] Application ready  ✓")
        print("[SANAD] Look at the edges to reveal buttons, dwell 1.5s to click")

    # ────────────────────────────────────────────────────────────────
    @staticmethod
    def _detect_screen_size():
        """Detect the primary monitor resolution. Falls back to 1920x1080."""
        # Method 1: try the screeninfo library
        try:
            from screeninfo import get_monitors
            m = get_monitors()[0]
            return m.width, m.height
        except Exception:
            pass

        # Method 2: parse xrandr output (Linux)
        try:
            import subprocess
            out = subprocess.check_output(
                ["xrandr", "--current"], stderr=subprocess.DEVNULL
            ).decode()
            for line in out.splitlines():
                if " connected" in line and "primary" in line:
                    # e.g. "eDP-1 connected primary 1920x1080+0+0 ..."
                    for token in line.split():
                        if "x" in token and "+" in token:
                            res = token.split("+")[0]
                            w, h = res.split("x")
                            return int(w), int(h)
                # Fallback: first connected with resolution
                if " connected" in line:
                    for token in line.split():
                        if "x" in token and "+" in token:
                            res = token.split("+")[0]
                            w, h = res.split("x")
                            return int(w), int(h)
        except Exception:
            pass

        # Method 3: xdpyinfo
        try:
            import subprocess
            out = subprocess.check_output(
                ["xdpyinfo"], stderr=subprocess.DEVNULL
            ).decode()
            for line in out.splitlines():
                if "dimensions:" in line:
                    # "  dimensions:    1920x1080 pixels ..."
                    res = line.strip().split()[1]
                    w, h = res.split("x")
                    return int(w), int(h)
        except Exception:
            pass

        # Fallback
        return 1920, 1080

    # ────────────────────────────────────────────────────────────────
    def _load_screen(self, name):
        if name in self.screens:
            self.current_screen_name = name
            btn_defs = self.screens[name].get_popup_buttons()
            self.popup.set_buttons(btn_defs)

    # ────────────────────────────────────────────────────────────────
    def _handle_action(self, action_id):
        """Process a button click action."""
        print(f"[ACTION] {action_id}")

        # Navigation between screens
        if action_id in self.screens:
            self._load_screen(action_id)
            self._notify(f"Switched to {self.screens[action_id].TITLE}", C.ACCENT_TEAL)
            return

        # Generic actions
        action_map = {
            "speed_up":      ("Speed increased", C.ACCENT_BLUE),
            "speed_down":    ("Speed decreased", C.ACCENT_BLUE),
            "stop":          ("STOPPED", C.ACCENT_RED),
            "emergency":     ("EMERGENCY ALERT!", C.ACCENT_RED),
            "call":          ("Calling caregiver...", C.ACCENT_GREEN),
            "call_family":   ("Calling family...", C.ACCENT_GREEN),
            "call_doctor":   ("Calling doctor...", C.ACCENT_RED),
            "alert_nurse":   ("Nurse alerted!", C.ACCENT_ORANGE),
            "turn_left":     ("Turning LEFT", C.ACCENT_BLUE),
            "turn_right":    ("Turning RIGHT", C.ACCENT_BLUE),
            "reverse":       ("Reversing", C.ACCENT_ORANGE),
            "reroute":       ("Calculating new route...", C.ACCENT_TEAL),
            "say_yes":       ('Said: "Yes"', C.ACCENT_GREEN),
            "say_no":        ('Said: "No"', C.ACCENT_RED),
            "water":         ('Said: "I need water"', C.ACCENT_BLUE),
            "help":          ('Said: "I need help"', C.ACCENT_ORANGE),
            "pain":          ('Said: "I feel pain"', C.ACCENT_RED),
            "log_symptom":   ("Symptom logged", C.ACCENT_PURPLE),
            "take_meds":     ("Medication reminder set", C.ACCENT_GREEN),
            "pain_level":    ("Pain level recorded", C.ACCENT_RED),
            "view_history":  ("Loading health history...", C.ACCENT_BLUE),
            "vol_up":        ("Volume increased", C.ACCENT_GREEN),
            "vol_down":      ("Volume decreased", C.ACCENT_GREEN),
            "bright_up":     ("Brightness increased", C.ACCENT_ORANGE),
            "bright_down":   ("Brightness decreased", C.ACCENT_ORANGE),
            "reset_settings":("Settings reset to defaults", C.ACCENT_PURPLE),
        }
        if action_id in action_map:
            msg, color = action_map[action_id]
            self._notify(msg, color)
        else:
            self._notify(f"Action: {action_id}", C.GRAY)

    # ────────────────────────────────────────────────────────────────
    def _notify(self, message, color=C.ACCENT_TEAL, duration=2.0):
        self._notification = (message, time.time() + duration, color)

    def _draw_notification(self, canvas):
        if self._notification is None:
            return
        msg, expire, color = self._notification
        if time.time() > expire:
            self._notification = None
            return
        h, w = canvas.shape[:2]
        # Semi-transparent banner at the top
        banner_h = 42
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 66), (w, 66 + banner_h), (20, 20, 25), -1)
        cv2.addWeighted(overlay, 0.85, canvas, 0.15, 0, canvas)
        cv2.rectangle(canvas, (0, 66), (w, 68), color, -1)
        # Text
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(msg, font, 0.55, 1)
        tx = (w - tw) // 2
        ty = 66 + (banner_h + th) // 2
        cv2.putText(canvas, msg, (tx, ty), font, 0.55, color, 1, cv2.LINE_AA)

    # ────────────────────────────────────────────────────────────────
    def _draw_cursor(self, canvas, gx, gy, eye_state):
        """Draw a smooth gaze cursor with trail."""
        h, w = canvas.shape[:2]
        cx = int(gx * w)
        cy = int(gy * h)

        # Trail
        self._cursor_trail.append((cx, cy, time.time()))
        # Prune old trail points
        now = time.time()
        self._cursor_trail = [(x, y, t) for x, y, t in self._cursor_trail
                              if now - t < 0.3]
        # Draw trail
        for i, (tx, ty, tt) in enumerate(self._cursor_trail):
            age = now - tt
            alpha = max(0.0, 1.0 - age / 0.3)
            r = max(1, int(4 * alpha))
            color = tuple(int(c * alpha) for c in C.ACCENT_TEAL)
            cv2.circle(canvas, (tx, ty), r, color, -1, cv2.LINE_AA)

        # Main cursor
        if eye_state == "open":
            cursor_color = C.ACCENT_TEAL
        else:
            cursor_color = C.ACCENT_RED

        # Outer ring
        cv2.circle(canvas, (cx, cy), 12, cursor_color, 2, cv2.LINE_AA)
        # Inner dot
        cv2.circle(canvas, (cx, cy), 3, cursor_color, -1, cv2.LINE_AA)
        # Crosshair lines
        cv2.line(canvas, (cx-18, cy), (cx-8, cy), cursor_color, 1, cv2.LINE_AA)
        cv2.line(canvas, (cx+8, cy), (cx+18, cy), cursor_color, 1, cv2.LINE_AA)
        cv2.line(canvas, (cx, cy-18), (cx, cy-8), cursor_color, 1, cv2.LINE_AA)
        cv2.line(canvas, (cx, cy+8), (cx, cy+18), cursor_color, 1, cv2.LINE_AA)

    # ────────────────────────────────────────────────────────────────
    def _draw_camera_thumbnail(self, canvas, frame):
        """Draw a small camera feed thumbnail in the corner for debugging."""
        h, w = canvas.shape[:2]
        thumb_w = 140
        thumb_h = 100
        margin = 8
        # Position: bottom-right, above status bar
        tx = w - thumb_w - margin
        ty = h - thumb_h - 44

        thumb = cv2.resize(frame, (thumb_w, thumb_h))
        # Border
        cv2.rectangle(canvas, (tx-2, ty-2), (tx+thumb_w+2, ty+thumb_h+2),
                      C.DIVIDER, 2)
        canvas[ty:ty+thumb_h, tx:tx+thumb_w] = thumb

    # ────────────────────────────────────────────────────────────────
    def run(self):
        """Main loop."""
        print("[SANAD] Running... Press 'q' or ESC to quit.\n")

        while True:
            # ── Get gaze ────────────────────────────────────────────
            result = self.gaze.process_frame()
            if result is None:
                continue
            gx, gy, eye_state, cam_frame = result

            # ── Draw current screen ─────────────────────────────────
            self.canvas = np.zeros((self.WIN_H, self.WIN_W, 3), dtype=np.uint8)
            screen = self.screens[self.current_screen_name]
            screen.draw(self.canvas)

            # ── Update and draw popups ──────────────────────────────
            action = self.popup.update(gx, gy, self.WIN_W, self.WIN_H)
            if action is not None:
                self._handle_action(action)

            self.popup.draw(self.canvas)

            # ── Notification ────────────────────────────────────────
            self._draw_notification(self.canvas)

            # ── Cursor ──────────────────────────────────────────────
            self._draw_cursor(self.canvas, gx, gy, eye_state)

            # ── Camera thumbnail ────────────────────────────────────
            self._draw_camera_thumbnail(self.canvas, cam_frame)

            # ── Display ─────────────────────────────────────────────
            cv2.imshow(self.WINDOW_NAME, self.canvas)

            # ── Keyboard ────────────────────────────────────────────
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):   # q or ESC
                break
            elif key == ord('h'):
                self._load_screen("home")
            elif key == ord('n'):
                self._load_screen("nav")
            elif key == ord('c'):
                self._load_screen("comm")
            elif key == ord('m'):
                self._load_screen("health")
            elif key == ord('s'):
                self._load_screen("settings")

        # ── Cleanup ─────────────────────────────────────────────────
        self.gaze.release()
        cv2.destroyAllWindows()
        print("[SANAD] Shut down cleanly.")


# ═══════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description="SANAD Eye-Controlled Pop-Up Display Demo"
    )
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera device index (default: 0)")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to face_landmarker.task model file")
    args = parser.parse_args()

    try:
        app = SANADApp(camera_index=args.camera, model_path=args.model)
        app.run()
    except FileNotFoundError as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
