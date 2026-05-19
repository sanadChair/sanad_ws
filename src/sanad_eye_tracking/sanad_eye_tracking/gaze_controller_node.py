#!/usr/bin/env python3
"""
gaze_controller_node.py – Translates raw gaze data into navigation commands.

Subscribes
----------
/eye_tracking/gaze_position   (std_msgs/String)
/eye_tracking/eye_state       (std_msgs/String)

Publishes
---------
/eye_tracking/command  (std_msgs/String)
    One of:
      • "click"       – user closed eyes for `click_hold_time` seconds

/eye_tracking/cursor_position  (std_msgs/String)
    Current cursor position as "x,y" (0.0 to 1.0).

Logic
-----
1. Cursor hold:      Cursor position tracks gaze position while eyes are open.
2. Freeze on close:  If eyes blink or close, cursor position freezes.
3. Eyes closed 2 s:  Continuous "closed" state for `click_hold_time` s → emit "click".
4. Cooldown:         1 s idle between any two commands to prevent spam.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GazeControllerNode(Node):
    """State machine that interprets gaze stream and eye events."""

    def __init__(self):
        super().__init__("gaze_controller")

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter("click_hold_time", 2.0)
        self.declare_parameter("command_cooldown", 1.0)

        self.click_hold     = self.get_parameter("click_hold_time").value
        self.cmd_cooldown   = self.get_parameter("command_cooldown").value

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(String, "/eye_tracking/gaze_position",
                                 self._gaze_cb, 10)
        self.create_subscription(String, "/eye_tracking/eye_state",
                                 self._eye_cb, 10)

        # ── Publishers ──────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(String, "/eye_tracking/command", 10)
        self.cursor_pub = self.create_publisher(String, "/eye_tracking/cursor_position", 10)

        # ── Eye-closed state ────────────────────────────────────────
        self._close_start_time = None
        self._close_triggered = False
        self._eyes_open = True

        # ── Command cooldown ────────────────────────────────────────
        self._last_command_time = 0.0

        # ── Cursor position (start in the centre) ──────────────────────
        self._cursor_x = 0.5
        self._cursor_y = 0.5
        self._publish_cursor_position()

        self.get_logger().info(
            f"GazeControllerNode started  ✓  Continuous Cursor Mode"
        )

    # ================================================================
    #  Callbacks
    # ================================================================

    def _gaze_cb(self, msg: String):
        """Handle gaze position updates. Update cursor if eyes are open."""
        if self._eyes_open:
            try:
                x_str, y_str = msg.data.split(',')
                self._cursor_x = float(x_str)
                self._cursor_y = float(y_str)
            except ValueError:
                self.get_logger().error(f"Failed to parse gaze position: {msg.data}")
                
        # Always publish the current cursor position, even if frozen
        self._publish_cursor_position()

    # ----------------------------------------------------------------
    def _eye_cb(self, msg: String):
        """Handle eye state updates (open / blink / closed)."""
        state = msg.data
        now = time.time()

        if state == "open":
            self._eyes_open = True
            self._close_start_time = None
            self._close_triggered = False
        else:
            self._eyes_open = False  # Freeze cursor on blink or closed

        # ── Prolonged closure detection (click) ───────────────────
        if state == "closed":
            if self._close_start_time is None:
                self._close_start_time = now
                self._close_triggered = False
            elif not self._close_triggered:
                elapsed = now - self._close_start_time
                if elapsed >= self.click_hold:
                    self._close_triggered = True
                    if self._emit_command("click"):
                        self.get_logger().info(f"👆 Eyes closed {self.click_hold} s → CLICK at ({self._cursor_x:.2f}, {self._cursor_y:.2f})")
        elif state == "blink":
            # Just freeze cursor, don't trigger click timer
            self._close_start_time = None
            self._close_triggered = False

    # ================================================================
    #  Helpers
    # ================================================================

    def _emit_command(self, command: str) -> bool:
        """Publish a command if cooldown has elapsed. Returns True if sent."""
        now = time.time()
        if now - self._last_command_time < self.cmd_cooldown:
            return False
        self._last_command_time = now
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        return True

    def _publish_cursor_position(self):
        msg = String()
        msg.data = f"{self._cursor_x:.4f},{self._cursor_y:.4f}"
        self.cursor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
