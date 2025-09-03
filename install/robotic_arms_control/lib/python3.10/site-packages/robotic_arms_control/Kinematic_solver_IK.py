#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
from urdf_parser_py.urdf import URDF

class RobotDynamics(Node):
    def __init__(self):
        super().__init__('robot_dynamics_node')

        # Load robot URDF from ROS2 parameter server
        robot_description_param = '/robot_description'
        if not self.has_parameter(robot_description_param):
            self.get_logger().info(f"Parameter {robot_description_param} not found. Trying from file...")
            urdf_file = '/home/vishal/UR5_robot/src/robotic_arms_control/robotic_arms_control/urdf/ur5.urdf'  # change this
        else:
            urdf_str = self.get_parameter(robot_description_param).get_parameter_value().string_value
            with open('temp_robot.urdf', 'w') as f:
                f.write(urdf_str)
            urdf_file = 'temp_robot.urdf'

        # Load robot model in Pinocchio
        self.get_logger().info(f"Loading URDF: {urdf_file}")
        self.robot = RobotWrapper.BuildFromURDF(urdf_file, package_dirs=["/path/to/your/packages"])
        
        # Gravity in world frame
        self.robot.model.gravity.linear[:] = [0, 0, -9.81]

        # Get neutral configuration
        self.q = pin.neutral(self.robot.model)
        self.dq = np.zeros(self.robot.model.nv)

        # Compute and print matrices
        self.compute_dynamics()

    def compute_dynamics(self):
        # Mass/Inertia matrix
        M = self.robot.mass(self.q)
        print("Mass/Inertia Matrix M(q):\n", M)

        # Coriolis/centrifugal matrix
        C = pin.crba(self.robot.model, self.robot.data, self.q)  # CRBA gives mass matrix
        # Use RNEA for Coriolis+gravity torque
        tau = pin.rnea(self.robot.model, self.robot.data, self.q, self.dq, np.zeros_like(self.dq))
        print("Coriolis+Gravity Torque (tau) at zero acceleration:\n", tau)

        # Gravitational vector G(q)
        G = pin.rnea(self.robot.model, self.robot.data, self.q, np.zeros(self.robot.model.nv), np.zeros(self.robot.model.nv))
        print("Gravitational vector G(q):\n", G)

        # Damping matrix B(q) from URDF (diagonal)
        B = []
        urdf_robot = URDF.from_xml_file('/path/to/your/robot.urdf')  # same URDF
        for joint in urdf_robot.joints:
            damping = joint.dynamics.damping if joint.dynamics else 0.0
            if joint.type != 'fixed':
                B.append(damping)
        B_matrix = np.diag(B)
        print("Damping matrix B(q):\n", B_matrix)

        # Stiffness (if specified in Gazebo tags)
        K = []  # Example, you can populate from <spring_stiffness> in URDF Gazebo tags
        K_matrix = np.diag(K)
        print("Stiffness matrix K(q):\n", K_matrix)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDynamics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
