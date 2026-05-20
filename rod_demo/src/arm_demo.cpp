#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("arm_demo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    auto node = rclcpp::Node::make_shared("arm_demo_node", "/arm", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(LOGGER, "Planning Group: %s", PLANNING_GROUP.c_str());
    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector:   %s", move_group.getEndEffectorLink().c_str());

    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(LOGGER, "Aktuelle Pose: x=%.3f y=%.3f z=%.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z);

    // ----------------------------------------------------------------
    // Posen definieren (Joint-Werte in Radiant)
    // Reihenfolge: joint_1 ... joint_6
    // ----------------------------------------------------------------
    std::vector<double> home  = { 0.0,  0.0, -1.5708, -1.5708,  0.0, 0.0};
    std::vector<double> zero  = { 0.0,  0.0,  0.0,     0.0,     0.0, 0.0};
    std::vector<double> pose1 = { 0.5, -0.3, -1.2,    -1.5,     0.2, 0.0};

    auto move_to_joints = [&](const std::vector<double>& joints, const std::string& name) {
        RCLCPP_INFO(LOGGER, "Fahre zu: %s", name.c_str());
        move_group.setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) ==
                        moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.execute(plan);
            RCLCPP_INFO(LOGGER, "Erfolgreich: %s", name.c_str());
        } else {
            RCLCPP_ERROR(LOGGER, "Planung fehlgeschlagen: %s", name.c_str());
        }
    };

    // ----------------------------------------------------------------
    // Ablauf
    // ----------------------------------------------------------------
    move_to_joints(home,  "Home");
    move_to_joints(pose1, "Pose 1");
    move_to_joints(home,  "Home");
    move_to_joints(zero,  "Zero");
    move_to_joints(home,  "Home");

    // ----------------------------------------------------------------
    // Kartesische Bewegung (Linear)
    // ----------------------------------------------------------------
    RCLCPP_INFO(LOGGER, "Kartesische Bewegung: 10cm nach oben");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose linear_pose = move_group.getCurrentPose().pose;
    linear_pose.position.z += 0.1;
    waypoints.push_back(linear_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, trajectory);
    if (fraction > 0.9) {
        move_group.execute(trajectory);
        RCLCPP_INFO(LOGGER, "Kartesische Bewegung erfolgreich (%.1f%%)", fraction * 100.0);
    } else {
        RCLCPP_WARN(LOGGER, "Kartesische Bewegung nur %.1f%% planbar", fraction * 100.0);
    }

    move_to_joints(home, "Home (Ende)");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
