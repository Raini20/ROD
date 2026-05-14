import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('scara_4')
    gz_package = get_package_share_directory('ros_gz_sim')
    controller_config = os.path.join(pkg, 'config', 'scara_controller.yaml')

    with open(os.path.join(pkg, 'urdf', 'SCARA_4.urdf'), 'r') as f:
        robot_description = f.read().replace('$(find scara_4)', pkg)

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'scara_4', 'share')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_package, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf', 'use_sim_time': 'True'}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            parameters=[{'name': 'scara', 'topic': 'robot_description', 'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/empty/model/scara/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            remappings=[('/world/empty/model/scara/joint_state', 'joint_states')],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['scara_controller', '--param-file', controller_config],
            parameters=[{'use_sim_time': True}]
        ),
    ])
