import rclpy
from rclpy.executors import MultiThreadedExecutor

from .calibration_node import CalibrationNode
from .fusion_node import FusionNode

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    cal = CalibrationNode()
    fusion = FusionNode()

    executor.add_node(cal)
    executor.add_node(fusion)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        cal.destroy_node()
        fusion.destroy_node()
        rclpy.shutdown()