from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    arm_urdf = os.path.join(
        get_package_share_directory('robot_arm_6dof_assembly'),
        'urdf', 'robot_arm_6dof_assembly.urdf')

    scara_urdf = os.path.join(
        get_package_share_directory('scara_4'),
        'urdf', 'SCARA_4.urdf')

    with open(arm_urdf, 'r') as f:
        arm_description = f.read()

    with open(scara_urdf, 'r') as f:
        scara_description = f.read()

    install_share = os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'robot_arm_6dof_assembly', 'share') + ':' + os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'scara_4', 'share')

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            install_share
        ),

        gz_launch,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_state_publisher',
            namespace='arm',
            parameters=[{'robot_description': arm_description}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='scara_state_publisher',
            namespace='scara',
            parameters=[{'robot_description': scara_description}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'arm', '-topic', '/arm/robot_description',
                       '-x', '0.0', '-y', '0.0', '-z', '0.0'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'scara', '-topic', '/scara/robot_description',
                       '-x', '1.5', '-y', '0.0', '-z', '0.0'],
            output='screen'
        ),
    ])
