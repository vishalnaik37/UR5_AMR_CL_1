import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Joint trajectory controller topic
        topic_name = "/joint_trajectory_controller/joint_trajectory"

        # Publisher for JointTrajectory messages
        self.publisher_ = self.create_publisher(JointTrajectory, topic_name, 10)

        # Timer for publishing at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Example joint names (replace with your robot’s joint names!)
        self.joint_names = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ]

        self.i = 0.0  # time counter

    def timer_callback(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()

        # Example: generate a smooth sine-wave trajectory
        point.positions = [0.5 * math.sin(self.i)] * len(self.joint_names)
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=2)

        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"Publishing point with positions: {point.positions}")

        self.i += 0.1  # increment time


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
