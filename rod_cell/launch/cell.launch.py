import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    arm_pkg = get_package_share_directory('robot_arm_6dof_assembly')
    scara_pkg = get_package_share_directory('scara_4')
    scene_pkg = get_package_share_directory('rod_scene')
    gz_package = get_package_share_directory('ros_gz_sim')

    with open(os.path.join(arm_pkg, 'urdf', 'robot_arm_6dof_assembly.urdf'), 'r') as f:
        arm_description = f.read().replace('$(find robot_arm_6dof_assembly)', arm_pkg)

    with open(os.path.join(scara_pkg, 'urdf', 'SCARA_4.urdf'), 'r') as f:
        scara_description = f.read().replace('$(find scara_4)', scara_pkg)

    gz_resource_path = ':'.join([
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'robot_arm_6dof_assembly', 'share'),
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'scara_4', 'share'),
        os.path.join(os.path.expanduser('~'), 'rod_ws', 'install', 'rod_scene', 'share'),
    ])

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
    toaster_mesh = os.path.join(scene_pkg, 'meshes', 'ToasterAssembly.glb')
    conveyor_mesh = os.path.join(scene_pkg, 'meshes', 'Conveyor.glb')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_package, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
            # launch_arguments={'gz_args': '-r empty.sdf --render-engine ogre2'}.items()
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
        ),

        # Roboter
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
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'arm', '-topic', '/arm/robot_description',
                       '-x', '-0.75', '-y', '0.0', '-z', '1.0'],
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'scara', '-topic', '/scara/robot_description',
                       '-x', '1.0', '-y', '0', '-z', '1.0'],
        ),

        # Säulen
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'column_arm', '-string',
                       make_static_sdf('column_arm', column_mesh, -0.75, 0, 0, 1.5708, 0, 0)],
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'column_scara', '-string',
                       make_static_sdf('column_scara', column_mesh, 1, 0, 0, 1.5708, 0, 0)],
        ),

        # Fixiereinheit
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'fixiereinheit', '-string',
                       make_static_sdf('fixiereinheit', fixier_mesh, 0, 0, 0)],
        ),

        # Toaster (auf dem Boden vorerst)
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'toaster', '-string',
                       make_static_sdf('toaster', toaster_mesh, -1.5, 0, 0)],
        ),

        # FB1 - Eingang Gehäuse (von links)
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'fb1', '-string',
                       make_static_sdf('fb1', conveyor_mesh, -0.75, 0.75, 0, 1.5708, 0, 0)],
        ),
        # FB2 - Eingang Deckel (von links, versetzt)
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'fb2', '-string',
                       make_static_sdf('fb2', conveyor_mesh, 0, 0.75, 0, 1.5708, 0, 0)],
        ),
        # FB3 - Ausgang (nach rechts)
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-name', 'fb3', '-string',
                       make_static_sdf('fb3', conveyor_mesh, 0, -0.75, 0, 1.5708, 0, 3.14159)],
        ),
    ])
