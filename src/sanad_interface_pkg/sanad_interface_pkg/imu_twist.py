#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION — edit these values to tune the controller
# ═════════════════════════════════════════════════════════════════════════════

# Which IMU axis to use for each motion.
# Valid values: "x", "y", or "z"
FORWARD_AXIS = "z"  # linear_acceleration axis → forward / backward
TURN_AXIS = "x"  # linear_acceleration axis → left / right

# Deadband: minimum deviation from neutral (m/s²) before any motion is sent.
# Set this just above your IMU's resting noise level.
DEADBAND = 0.20  # m/s²

# Max lean: the deviation (m/s²) that maps to FULL speed.
# Leaning beyond this is clamped to full speed.
MAX_LEAN = 1.00  # m/s²

# Output speed limits
MAX_LINEAR = 0.30  # m/s   — top forward / backward speed
MAX_ANGULAR = 0.40  # rad/s — top turning speed

# Slew rates: how fast the output speed is allowed to ramp per second.
# Lower = smoother but slower to respond. Higher = faster but may feel jerky.
SLEW_LINEAR = 0.10  # m/s  per second
SLEW_ANGULAR = 0.15  # rad/s per second

# Calibration: how many seconds to collect the neutral baseline at startup.
CALIB_DURATION = 5.0  # seconds

# ═════════════════════════════════════════════════════════════════════════════


VALID_AXES = ("x", "y", "z")


class ImuToCmdVel(Node):
    def __init__(self):
        super().__init__("imu_to_cmdvel")

        # Validate axis config
        fwd = FORWARD_AXIS.strip().lower()
        turn = TURN_AXIS.strip().lower()

        if fwd not in VALID_AXES:
            self.get_logger().error(
                f"Invalid FORWARD_AXIS '{fwd}'. Must be x/y/z. Defaulting to 'y'."
            )
            fwd = "y"
        if turn not in VALID_AXES:
            self.get_logger().error(
                f"Invalid TURN_AXIS '{turn}'. Must be x/y/z. Defaulting to 'z'."
            )
            turn = "z"
        if fwd == turn:
            self.get_logger().error(
                "FORWARD_AXIS and TURN_AXIS cannot be the same! Defaulting to y/z."
            )
            fwd, turn = "y", "z"

        self.forward_axis = fwd
        self.turn_axis = turn

        # ── Calibration state ─────────────────────────────────────────────────
        self._calib_start = None
        self._calib_done = False
        self._calib_fwd = []
        self._calib_trn = []
        self._neutral_fwd = 0.0
        self._neutral_trn = 0.0
        self._last_countdown = -1

        # ── Runtime state ─────────────────────────────────────────────────────
        self._last_linear = 0.0
        self._last_angular = 0.0
        self._last_time = None

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info(
            f"\n"
            f"  IMU Wheelchair Controller\n"
            f"  ─────────────────────────────────────\n"
            f"  FORWARD_AXIS : {self.forward_axis}      (forward / backward)\n"
            f"  TURN_AXIS    : {self.turn_axis}      (left / right)\n"
            f"  DEADBAND     : {DEADBAND:.3f} m/s²\n"
            f"  MAX_LEAN     : {MAX_LEAN:.3f} m/s²\n"
            f"  MAX_LINEAR   : {MAX_LINEAR:.3f} m/s\n"
            f"  MAX_ANGULAR  : {MAX_ANGULAR:.3f} rad/s\n"
            f"  ─────────────────────────────────────\n"
            f"  Hold still — calibrating for {CALIB_DURATION:.0f} seconds..."
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_axis(self, msg: Imu, axis: str) -> float:
        if axis == "x":
            return msg.linear_acceleration.x
        elif axis == "y":
            return msg.linear_acceleration.y
        else:
            return msg.linear_acceleration.z

    def _apply_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < DEADBAND else value

    def _normalize(self, value: float) -> float:
        """Map error from [DEADBAND, MAX_LEAN] → [0, 1] linearly, clamped."""
        if value == 0.0:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        scaled = (abs(value) - DEADBAND) / max(1e-6, MAX_LEAN - DEADBAND)
        return sign * min(1.0, max(0.0, scaled))

    def _slew(self, current: float, target: float, max_step: float) -> float:
        delta = target - current
        delta = max(-max_step, min(max_step, delta))
        return current + delta

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    # ── Calibration ───────────────────────────────────────────────────────────

    def _run_calibration(self, now, fwd_val: float, trn_val: float) -> bool:
        if self._calib_start is None:
            self._calib_start = now
            self.get_logger().info("Calibration started — please keep your head still.")

        self._calib_fwd.append(fwd_val)
        self._calib_trn.append(trn_val)

        elapsed = (now - self._calib_start).nanoseconds * 1e-9
        remaining = int(CALIB_DURATION - elapsed) + 1

        if remaining != self._last_countdown:
            self._last_countdown = remaining
            self.get_logger().info(f"Calibrating... {remaining}s remaining")

        if elapsed < CALIB_DURATION:
            return False

        n = len(self._calib_fwd)
        if n < 10:
            self.get_logger().warn(
                f"Only {n} samples — too few. Restarting calibration..."
            )
            self._calib_start = None
            self._calib_fwd.clear()
            self._calib_trn.clear()
            self._last_countdown = -1
            return False

        self._neutral_fwd = sum(self._calib_fwd) / n
        self._neutral_trn = sum(self._calib_trn) / n

        self.get_logger().info(
            f"Calibration done! {n} samples. "
            f"Neutral -> {self.forward_axis}={self._neutral_fwd:.4f}  "
            f"{self.turn_axis}={self._neutral_trn:.4f}. "
            "Wheelchair is ready to drive."
        )
        return True

    # ── Main callback ─────────────────────────────────────────────────────────

    def imu_callback(self, msg: Imu):
        now = self.get_clock().now()
        fwd_val = self._get_axis(msg, self.forward_axis)
        trn_val = self._get_axis(msg, self.turn_axis)

        # Phase 1: calibration
        if not self._calib_done:
            self.pub.publish(Twist())
            self._calib_done = self._run_calibration(now, fwd_val, trn_val)
            return

        # Phase 2: control
        err_fwd = self._apply_deadband(fwd_val - self._neutral_fwd)
        err_trn = self._apply_deadband(trn_val - self._neutral_trn)

        # Normalize to [-1, 1] then scale to speed limits
        # Lean forward (err_fwd > 0) -> drive forward  (linear.x  > 0)
        # Lean left    (err_trn > 0) -> turn left       (angular.z > 0)
        # Flip sign below if a direction feels inverted on your hardware.
        target_linear = self._clamp(self._normalize(err_fwd) * MAX_LINEAR, MAX_LINEAR)
        target_angular = self._clamp(
            self._normalize(err_trn) * MAX_ANGULAR, MAX_ANGULAR
        )

        # Slew-rate limiting
        dt = (
            0.0
            if self._last_time is None
            else max(0.0, (now - self._last_time).nanoseconds * 1e-9)
        )

        cmd_linear = self._slew(self._last_linear, target_linear, SLEW_LINEAR * dt)
        cmd_angular = self._slew(self._last_angular, target_angular, SLEW_ANGULAR * dt)

        cmd = Twist()
        cmd.linear.x = cmd_linear
        cmd.angular.z = cmd_angular
        self.pub.publish(cmd)

        self._last_linear = cmd_linear
        self._last_angular = cmd_angular
        self._last_time = now


# ─────────────────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = ImuToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
