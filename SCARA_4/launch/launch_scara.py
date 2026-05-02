from launch import LaunchDescription
from launch_ros.actions import Node

# Pfad zur fixed URDF (mit absoluten file:// Mesh-Pfaden)
urdf_path = "/mnt/c/Users/Raini/Desktop/FH/Master/2/ROD/SCARA/SCARA_4/urdf/SCARA_test.urdf"

with open(urdf_path, "r") as f:
    robot_description = f.read()

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-f", "world"],
        ),
    ])
