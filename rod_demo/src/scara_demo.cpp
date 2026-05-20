#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("scara_demo");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    auto node = rclcpp::Node::make_shared("scara_demo_node", "/scara", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    static const std::string PLANNING_GROUP = "scara";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(LOGGER, "Planning Group: %s", PLANNING_GROUP.c_str());
    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector:   %s", move_group.getEndEffectorLink().c_str());

    // ----------------------------------------------------------------
    // Posen definieren
    // joint_3 ist prismatisch (Meter): 0.0 = oben, -0.4 = unten
    // Reihenfolge: joint_1, joint_2, joint_3, joint_4
    // ----------------------------------------------------------------
    std::vector<double> home       = { 0.0,  0.0,  0.0,  0.0};
    std::vector<double> screw_pose = { 0.3, -0.3, -0.2,  0.0};
    std::vector<double> side_pose  = {-0.5,  0.2,  0.0,  0.5};

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
    move_to_joints(home,       "Home");
    move_to_joints(screw_pose, "Schraubpose");
    move_to_joints(home,       "Home");
    move_to_joints(side_pose,  "Seitliche Pose");
    move_to_joints(home,       "Home (Ende)");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
