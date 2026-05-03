# sanad_perception/tracker_node.py
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

from .tracker import TrackManager, Track


class TrackerNode(Node):
    """
    Subscribes : /detections      (Detection3DArray)
    Publishes  : /tracked_human   (PointStamped)  ← consumed by controller
                 /track_markers   (MarkerArray)   ← RViz2 debug
                 /active_track_id (Int32)         ← operator monitor

    ROS 2 params (tunable live with ros2 param set):
        min_hits    – frames to confirm a track      default 3
        max_misses  – frames before track deletion   default 8
        gate_eucl   – Euclidean gate in m            default 2.0
        gate_mahal  – Mahalanobis gate               default 4.0
        q_pos_var   – Kalman position process noise  default 0.02
        q_vel_var   – Kalman velocity process noise  default 0.10
        r_var       – Kalman measurement noise       default 0.05
    """

    QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    def __init__(self):
        super().__init__('tracker_node')

        # ── Declare tunable params ────────────────────────────────────────────
        self.declare_parameter('min_hits',   3)
        self.declare_parameter('max_misses', 8)
        self.declare_parameter('gate_eucl',  2.0)
        self.declare_parameter('gate_mahal', 4.0)
        self.declare_parameter('q_pos_var',  0.02)
        self.declare_parameter('q_vel_var',  0.10)
        self.declare_parameter('r_var',      0.05)

        # ── Build tracker ─────────────────────────────────────────────────────
        self._mgr = TrackManager()
        self._apply_params()   # push declared params into TrackManager

        # ── Timing ───────────────────────────────────────────────────────────
        self._last_stamp = None   # rclpy.time.Time of previous detection msg

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Detection3DArray, '/detections',
            self._on_detections, self.QOS)

        # Operator override: publish an Int32 to /force_target_id to lock
        # the tracker onto a specific track ID
        self.create_subscription(
            Int32, '/force_target_id',
            self._on_force_target, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_human  = self.create_publisher(
            PointStamped, '/tracked_human',   10)
        self._pub_id     = self.create_publisher(
            Int32, '/active_track_id',        10)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/track_markers',    10)

        self.get_logger().info('Tracker node ready.')

    # ── Main callback ─────────────────────────────────────────────────────────

    def _on_detections(self, msg: Detection3DArray):
        # ── Compute dt from message timestamps (robust to missed frames) ──────
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        if self._last_stamp is None:
            dt = 0.1   # default for first frame
        else:
            dt = (now - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = now

        # ── Re-read live params every frame (cheap dict lookup) ───────────────
        self._apply_params()

        # ── Extract [px, py] from each detection ──────────────────────────────
        measurements = [
            np.array([d.bbox.center.position.x,
                      d.bbox.center.position.y], dtype=np.float64)
            for d in msg.detections
        ]

        # ── Run tracker ───────────────────────────────────────────────────────
        confirmed = self._mgr.update(measurements, dt)
        target    = self._mgr.get_target()

        # ── Publish tracked human position ────────────────────────────────────
        if target is not None:
            pt = PointStamped()
            pt.header.stamp    = msg.header.stamp
            pt.header.frame_id = msg.header.frame_id
            pt.point.x = float(target.position[0])
            pt.point.y = float(target.position[1])
            pt.point.z = 0.0
            self._pub_human.publish(pt)

            id_msg = Int32()
            id_msg.data = target.track_id
            self._pub_id.publish(id_msg)

            self.get_logger().debug(
                f'Target id={target.track_id}  '
                f'pos=[{target.position[0]:.2f},{target.position[1]:.2f}]  '
                f'spd={target.speed:.2f}m/s  '
                f'hits={target.hits}  misses={target.misses}')

        # ── Publish RViz2 markers for all confirmed tracks ────────────────────
        self._publish_markers(confirmed, target, msg.header)

    # ── Force-target override ─────────────────────────────────────────────────

    def _on_force_target(self, msg: Int32):
        self._mgr.force_target(msg.data)
        self.get_logger().info(f'Operator locked target to track {msg.data}')

    # ── RViz2 visualisation ───────────────────────────────────────────────────

    def _publish_markers(self, confirmed, target, header):
        ma = MarkerArray()

        # Delete stale markers from last frame
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, trk in enumerate(confirmed):
            is_target = (target is not None and
                         trk.track_id == target.track_id)

            # ── Cylinder body ─────────────────────────────────────────────────
            cyl = Marker()
            cyl.header     = header
            cyl.ns         = 'tracks'
            cyl.id         = i * 3
            cyl.type       = Marker.CYLINDER
            cyl.action     = Marker.ADD
            cyl.pose.position.x = float(trk.position[0])
            cyl.pose.position.y = float(trk.position[1])
            cyl.pose.position.z = 0.85
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 0.5
            cyl.scale.y = 0.5
            cyl.scale.z = 1.7
            # Target → green, others → cyan
            if is_target:
                cyl.color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.6)
            else:
                cyl.color = ColorRGBA(r=0.2, g=0.7, b=1.0, a=0.3)
            cyl.lifetime = Duration(sec=0, nanosec=400_000_000)
            ma.markers.append(cyl)

            # ── Velocity arrow ────────────────────────────────────────────────
            arr = Marker()
            arr.header     = header
            arr.ns         = 'velocities'
            arr.id         = i * 3 + 1
            arr.type       = Marker.ARROW
            arr.action     = Marker.ADD
            arr.pose.position.x = float(trk.position[0])
            arr.pose.position.y = float(trk.position[1])
            arr.pose.position.z = 1.8
            arr.pose.orientation.w = 1.0
            arr.scale.x = float(trk.speed) * 0.5 + 0.01  # length = speed
            arr.scale.y = 0.05
            arr.scale.z = 0.05
            arr.color    = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
            arr.lifetime = Duration(sec=0, nanosec=400_000_000)
            ma.markers.append(arr)

            # ── Track ID text label ───────────────────────────────────────────
            txt = Marker()
            txt.header     = header
            txt.ns         = 'labels'
            txt.id         = i * 3 + 2
            txt.type       = Marker.TEXT_VIEW_FACING
            txt.action     = Marker.ADD
            txt.pose.position.x = float(trk.position[0])
            txt.pose.position.y = float(trk.position[1])
            txt.pose.position.z = 2.0
            txt.pose.orientation.w = 1.0
            txt.scale.z    = 0.2
            txt.text       = (f'ID:{trk.track_id} '
                              f'[{"TARGET" if is_target else "track"}]')
            txt.color      = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.lifetime   = Duration(sec=0, nanosec=400_000_000)
            ma.markers.append(txt)

        self._pub_markers.publish(ma)

    # ── Apply ROS 2 params → TrackManager ────────────────────────────────────

    def _apply_params(self):
        m = self._mgr
        m.MIN_HITS   = int(self.get_parameter('min_hits').value)
        m.MAX_MISSES = int(self.get_parameter('max_misses').value)
        m.GATE_EUCL  = float(self.get_parameter('gate_eucl').value)
        m.GATE_MAHAL = float(self.get_parameter('gate_mahal').value)

        # Push noise params into every existing track
        for t in m.all_tracks:
            t.Q_pos_var = float(self.get_parameter('q_pos_var').value)
            t.Q_vel_var = float(self.get_parameter('q_vel_var').value)
            t.R_var     = float(self.get_parameter('r_var').value)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()