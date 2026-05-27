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

    auto node = rclcpp::Node::make_shared("arm_demo_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    // MoveGroup explizit auf /arm Namespace zeigen
    moveit::planning_interface::MoveGroupInterface::Options opts(
        "arm",               // planning group
        "robot_description", // robot description topic
        "/arm"               // move_group namespace
    );

    moveit::planning_interface::MoveGroupInterface move_group(node, opts);

    move_group.setMaxVelocityScalingFactor(1);
    move_group.setMaxAccelerationScalingFactor(1);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector:   %s", move_group.getEndEffectorLink().c_str());

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

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
