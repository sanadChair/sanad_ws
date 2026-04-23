#!/usr/bin/env python3
"""
gaze_controller_node.py – Translates raw gaze data into navigation commands.

Subscribes
----------
/eye_tracking/gaze_direction  (std_msgs/String)
/eye_tracking/eye_state       (std_msgs/String)

Publishes
---------
/eye_tracking/command  (std_msgs/String)
    One of:
      • "move_north"  – gaze held north for `direction_hold_time` seconds
      • "move_south"  – gaze held south  …
      • "move_east"   – gaze held east   …
      • "move_west"   – gaze held west   …
      • "click"       – gaze held at center for `center_hold_time` seconds
      • "go_back"     – user closed eyes for `close_hold_time` seconds

/eye_tracking/grid_position  (std_msgs/String)
    Current grid position as "row,col" (0-indexed).

Logic
-----
1. Direction hold:   If gaze stays in a cardinal direction for 3 s
                     continuously → emit move_* and shift grid cursor.
2. Center hold:      If gaze stays at center for 3 s → emit "click"
                     (select the currently highlighted window).
3. Eyes closed 3 s:  Continuous "closed" state for 3 s → emit "go_back".
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
        self.declare_parameter("direction_hold_time", 3.0)
        self.declare_parameter("center_hold_time", 3.0)
        self.declare_parameter("close_hold_time", 3.0)
        self.declare_parameter("command_cooldown", 1.0)
        self.declare_parameter("grid_rows", 3)
        self.declare_parameter("grid_cols", 3)

        self.dir_hold       = self.get_parameter("direction_hold_time").value
        self.center_hold    = self.get_parameter("center_hold_time").value
        self.close_hold     = self.get_parameter("close_hold_time").value
        self.cmd_cooldown   = self.get_parameter("command_cooldown").value
        self.grid_rows      = self.get_parameter("grid_rows").value
        self.grid_cols      = self.get_parameter("grid_cols").value

        # ── Subscribers ─────────────────────────────────────────────
        self.create_subscription(String, "/eye_tracking/gaze_direction",
                                 self._gaze_cb, 10)
        self.create_subscription(String, "/eye_tracking/eye_state",
                                 self._eye_cb, 10)

        # ── Publishers ──────────────────────────────────────────────
        self.cmd_pub  = self.create_publisher(String, "/eye_tracking/command", 10)
        self.grid_pub = self.create_publisher(String, "/eye_tracking/grid_position", 10)

        # ── Gaze-hold state ─────────────────────────────────────────
        self._current_dir = "center"
        self._dir_start_time = time.time()
        self._dir_triggered = False      # avoid repeat triggers

        # ── Eye-closed state ────────────────────────────────────────
        self._close_start_time = None
        self._close_triggered = False

        # ── Command cooldown ────────────────────────────────────────
        self._last_command_time = 0.0

        # ── Grid cursor (start in the centre) ──────────────────────
        self._row = self.grid_rows // 2
        self._col = self.grid_cols // 2
        self._publish_grid_position()

        self.get_logger().info(
            f"GazeControllerNode started  ✓  Grid {self.grid_rows}×{self.grid_cols}, "
            f"cursor at ({self._row},{self._col})"
        )

    # ================================================================
    #  Callbacks
    # ================================================================

    def _gaze_cb(self, msg: String):
        """Handle gaze direction updates.

        • Cardinal direction held for `direction_hold_time` → move
        • Center held for `center_hold_time` → click
        """
        direction = msg.data  # "north", "south", "east", "west", "center"
        now = time.time()

        if direction != self._current_dir:
            # Direction changed → reset timer
            self._current_dir = direction
            self._dir_start_time = now
            self._dir_triggered = False
            return

        # Same direction – check if we've been holding long enough
        if self._dir_triggered:
            return

        elapsed = now - self._dir_start_time

        if direction == "center":
            # ── Center hold → click ─────────────────────────────────
            if elapsed >= self.center_hold:
                self._dir_triggered = True
                if self._emit_command("click"):
                    self.get_logger().info("👆  Center hold 3 s → CLICK")
        else:
            # ── Cardinal direction hold → move ──────────────────────
            if elapsed >= self.dir_hold:
                self._dir_triggered = True
                self._handle_direction_move(direction)

    # ----------------------------------------------------------------
    def _eye_cb(self, msg: String):
        """Handle eye state updates (open / blink / closed)."""
        state = msg.data
        now = time.time()

        # ── Prolonged closure detection (go_back) ───────────────────
        if state == "closed":
            if self._close_start_time is None:
                self._close_start_time = now
                self._close_triggered = False
            elif not self._close_triggered:
                elapsed = now - self._close_start_time
                if elapsed >= self.close_hold:
                    self._close_triggered = True
                    if self._emit_command("go_back"):
                        self.get_logger().info("🔙  Eyes closed 3 s → GO BACK")
        else:
            # Eyes reopened
            self._close_start_time = None
            self._close_triggered = False

    # ================================================================
    #  Helpers
    # ================================================================

    def _handle_direction_move(self, direction: str):
        """Move the grid cursor and emit a move command."""
        old_row, old_col = self._row, self._col

        if direction == "north":
            self._row = max(0, self._row - 1)
        elif direction == "south":
            self._row = min(self.grid_rows - 1, self._row + 1)
        elif direction == "west":
            self._col = max(0, self._col - 1)
        elif direction == "east":
            self._col = min(self.grid_cols - 1, self._col + 1)

        cmd = f"move_{direction}"
        if not self._emit_command(cmd):
            # Cooldown active — revert cursor move
            self._row, self._col = old_row, old_col
            return
        self._publish_grid_position()

        self.get_logger().info(
            f"⬛  {cmd}  ({old_row},{old_col}) → ({self._row},{self._col})"
        )

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

    def _publish_grid_position(self):
        msg = String()
        msg.data = f"{self._row},{self._col}"
        self.grid_pub.publish(msg)


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
