from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("scara", package_name="scara_moveit")
        .to_moveit_configs()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", str(moveit_config.package_path / "config" / "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
            {"move_group_namespace": "/scara"},
        ],
    )

    return LaunchDescription([rviz_node])
