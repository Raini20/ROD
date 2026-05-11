import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    arm_pkg = get_package_share_directory('robot_arm_6dof_assembly')
    scara_pkg = get_package_share_directory('scara_4')
    gz_package = get_package_share_directory('ros_gz_sim')

    with open(os.path.join(arm_pkg, 'urdf', 'robot_arm_6dof_assembly.urdf'), 'r') as f:
        arm_description = f.read().replace('$(find robot_arm_6dof_assembly)', arm_pkg)

    with open(os.path.join(scara_pkg, 'urdf', 'SCARA_4.urdf'), 'r') as f:
        scara_description = f.read().replace('$(find scara_4)', scara_pkg)

    gz_resource_path = (
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'robot_arm_6dof_assembly', 'share') + ':' +
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'scara_4', 'share')
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_package, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_state_publisher',
            namespace='arm',
            parameters=[{'robot_description': arm_description}],
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='scara_state_publisher',
            namespace='scara',
            parameters=[{'robot_description': scara_description}],
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'arm', '-topic', '/arm/robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'scara', '-topic', '/scara/robot_description',
                       '-x', '1.5', '-y', '0.0', '-z', '0.0'],
        ),
    ])
