import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    arm_pkg = get_package_share_directory('robot_arm_6dof_assembly')
    scara_pkg = get_package_share_directory('scara_4')
    scene_pkg = get_package_share_directory('rod_scene')
    gz_package = get_package_share_directory('ros_gz_sim')

    arm_controller_joints_config = os.path.join(arm_pkg, 'config', 'arm_controller_joints.yaml')
    scara_controller_joints_config = os.path.join(scara_pkg, 'config', 'scara_controller_joints.yaml')

    with open(os.path.join(arm_pkg, 'urdf', 'robot_arm_6dof_assembly.urdf'), 'r') as f:
        arm_description = f.read().replace('$(find robot_arm_6dof_assembly)', arm_pkg)

    with open(os.path.join(scara_pkg, 'urdf', 'SCARA_4.urdf'), 'r') as f:
        scara_description = f.read().replace('$(find scara_4)', scara_pkg)

    # Namespace in gz_ros2_control Plugin injizieren
    arm_description = re.sub(
        r'(name="gz_ros2_control::GazeboSimROS2ControlPlugin">)',
        r'\1\n      <ros><namespace>/arm</namespace></ros>',
        arm_description
    )
    scara_description = re.sub(
        r'(name="gz_ros2_control::GazeboSimROS2ControlPlugin">)',
        r'\1\n      <ros><namespace>/scara</namespace></ros>',
        scara_description
    )

    gz_resource_path = ':'.join([
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'robot_arm_6dof_assembly', 'share'),
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'scara_4', 'share'),
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'rod_scene', 'share'),
    ])

    def make_dynamic_sdf(name, mesh_path, x, y, z, roll=0, pitch=0, yaw=0):
        return f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="{name}">
        <static>false</static>
        <link name="link">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <inertial><mass>0.1</mass></inertial>
        <visual name="visual"><geometry><mesh><uri>{mesh_path}</uri></mesh></geometry></visual>
        </link>
    </model>
    </sdf>"""

    def make_static_sdf(name, mesh_path, x, y, z, roll=0, pitch=0, yaw=0):
        return f"""<?xml version="1.0"?>
    <sdf version="1.8">
    <model name="{name}">
        <static>true</static>
        <link name="link">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <visual name="visual">
            <geometry>
            <mesh><uri>{mesh_path}</uri></mesh>
            </geometry>
        </visual>
        </link>
    </model>
    </sdf>"""

    column_mesh = os.path.join(scene_pkg, 'meshes', 'column_robot_arm_6dof.glb')
    fixier_mesh = os.path.join(scene_pkg, 'meshes', 'FixiereinheitAssembly.glb')
    conveyor_mesh = os.path.join(scene_pkg, 'meshes', 'Conveyor.glb')
    toaster_shell_mesh = os.path.join(scene_pkg, 'meshes', 'ToasterShell.glb')
    toaster_innen_mesh = os.path.join(scene_pkg, 'meshes', 'ToasterInnen.glb')
    schraube_mesh = os.path.join(scene_pkg, 'meshes', 'Schraube.glb')

    # Schutzzaun-Meshes
    schutzzaun_mesh  = os.path.join(scene_pkg, 'meshes', 'Schutzzaun.glb')
    zaun_tuer_mesh   = os.path.join(scene_pkg, 'meshes', 'Schutzzaun_Tuer.glb')


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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
        ),

        # Arm
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='arm_state_publisher',
            namespace='arm',
            parameters=[{'robot_description': arm_description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'arm', '-topic', '/arm/robot_description',
                       '-x', '-0.75', '-y', '0.0', '-z', '1.0'],
        ),

        # SCARA
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='scara_state_publisher',
            namespace='scara',
            parameters=[{'robot_description': scara_description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'scara', '-topic', '/scara/robot_description',
                       '-x', '1.0', '-y', '0.0', '-z', '1.0'],
        ),

        # Controller (mit Delay damit Gazebo fertig ist)
        TimerAction(period=8.0, actions=[
            Node(
                package='controller_manager', executable='spawner',
                namespace='arm',
                arguments=['joint_state_broadcaster'],
            ),
            Node(
                package='controller_manager', executable='spawner',
                namespace='arm',
                arguments=['arm_controller',
                           '--param-file', arm_controller_joints_config],
            ),
            Node(
                package='controller_manager', executable='spawner',
                namespace='scara',
                arguments=['joint_state_broadcaster'],
            ),
            Node(
                package='controller_manager', executable='spawner',
                namespace='scara',
                arguments=['scara_controller',
                           '--param-file', scara_controller_joints_config],
            ),
        ]),

        # Szenenobjekte
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'column_arm', '-string',
                        make_static_sdf('column_arm', column_mesh, -0.75, 0, 0, 1.5708, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'column_scara', '-string',
                        make_static_sdf('column_scara', column_mesh, 1, 0, 0, 1.5708, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'fixiereinheit', '-string',
                        make_static_sdf('fixiereinheit', fixier_mesh, 0, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'fb1', '-string',
                        make_static_sdf('fb1', conveyor_mesh, -0.75, 0.3, 0, 1.5708, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'fb2', '-string',
                        make_static_sdf('fb2', conveyor_mesh, 0, 0.3, 0, 1.5708, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'fb3', '-string',
                        make_static_sdf('fb3', conveyor_mesh, 0, -0.3, 0, 1.5708, 0, 3.14159)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'toaster_shell', '-x', '-0.75', '-y', '0.45', '-z', '1.0', '-string',
                        make_dynamic_sdf('toaster_shell', toaster_shell_mesh, -0.75, 0.45, 1.0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'toaster_innen', '-x', '0.0', '-y', '0.45', '-z', '1.168', '-R', '3.14159', '-string',
                        make_dynamic_sdf('toaster_innen', toaster_innen_mesh, 0.0, 0.45, 1.168, 3.14159, 0, 0)]),
        #Node(package='ros_gz_sim', executable='create',
        #     arguments=['-name', 'output_shell', '-string',
        #                make_static_sdf('output_shell', toaster_shell_mesh, 0.0, -0.45, 1.0)]),
        #Node(package='ros_gz_sim', executable='create',
        #     arguments=['-name', 'output_innen', '-string',
        #                make_static_sdf('output_innen', toaster_innen_mesh, 0.0, -0.45, 1.0)]),
        Node(package='ros_gz_sim', executable='create',
            arguments=['-name', 'schraube_1', '-x',  '0.090', '-y', '0.505', '-z', '1.168', '-R', '3.14159', '-string',
                        make_dynamic_sdf('schraube_1', schraube_mesh, 0.090, 0.505, 1.168)]),
        Node(package='ros_gz_sim', executable='create',
            arguments=['-name', 'schraube_2', '-x', '-0.090', '-y', '0.505', '-z', '1.168', '-R', '3.14159', '-string',
                        make_dynamic_sdf('schraube_2', schraube_mesh, -0.090, 0.505, 1.168)]),
        Node(package='ros_gz_sim', executable='create',
            arguments=['-name', 'schraube_3', '-x',  '0.090', '-y', '0.395', '-z', '1.168', '-R', '3.14159', '-string',
                        make_dynamic_sdf('schraube_3', schraube_mesh, 0.090, 0.395, 1.168)]),
        Node(package='ros_gz_sim', executable='create',
            arguments=['-name', 'schraube_4', '-x', '-0.090', '-y', '0.395', '-z', '1.168', '-R', '3.14159', '-string',
                        make_dynamic_sdf('schraube_4', schraube_mesh, -0.090, 0.395, 1.168)]),

        # ── Schutzzaun ──────────────────────────────────────────────────────────
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'schutzzaun', '-string',
                        make_static_sdf('schutzzaun', schutzzaun_mesh, 0, 0, 0)]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'schutzzaun_tuer', '-string',
                        make_static_sdf('schutzzaun_tuer', zaun_tuer_mesh, -2.00, 0.40, 1.10)]),
    ])