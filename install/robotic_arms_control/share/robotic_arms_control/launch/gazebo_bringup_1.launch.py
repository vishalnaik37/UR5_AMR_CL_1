from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_description = get_package_share_directory('robotic_arms_control')
    urdf_file = os.path.join(pkg_description, "urdf", "ur5.urdf")

    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",   # must match <robot_param_node> in URDF
        output="screen",
        parameters=[{'robot_description': urdf_content}]
    )

    # Gazebo bringup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    # Spawn robot into Gazebo
    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ur5", "-topic", "robot_description"],
        output="screen"
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])
