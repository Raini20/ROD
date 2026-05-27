import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_moveit_pkg = get_package_share_directory('arm_moveit')
    arm_pkg = get_package_share_directory('robot_arm_6dof_assembly')

    with open(os.path.join(arm_pkg, 'urdf', 'robot_arm_6dof_assembly.urdf'), 'r') as f:
        urdf = f.read().replace('$(find robot_arm_6dof_assembly)', arm_pkg)

    with open(os.path.join(arm_moveit_pkg, 'config', 'knickarm_6dof.srdf'), 'r') as f:
        srdf = f.read()

    return LaunchDescription([
        Node(
            package='rod_demo',
            executable='arm_demo',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': urdf},
                {'robot_description_semantic': srdf},
                os.path.join(get_package_share_directory('rod_demo'),
                             'config', 'kinematics_ros2.yaml'),
            ],
        )
    ])
