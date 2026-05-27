#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

    moveit::planning_interface::MoveGroupInterface::Options opts(
        "arm", "robot_description", "/arm"
    );
    moveit::planning_interface::MoveGroupInterface move_group(node, opts);

    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(LOGGER, "Planning Frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector:   %s", move_group.getEndEffectorLink().c_str());

    // ----------------------------------------------------------------
    // Hilfsfunktion: Joint-Target
    // ----------------------------------------------------------------
    auto move_to_joints = [&](const std::vector<double>& joints, const std::string& name) {
        RCLCPP_INFO(LOGGER, "Fahre zu: %s", name.c_str());
        move_group.setJointValueTarget(joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.execute(plan);
            RCLCPP_INFO(LOGGER, "Erfolgreich: %s", name.c_str());
        } else {
            RCLCPP_ERROR(LOGGER, "Planung fehlgeschlagen: %s", name.c_str());
        }
    };

    // ----------------------------------------------------------------
    // Hilfsfunktion: Kartesische Pose (Position + Quaternion)
    // ----------------------------------------------------------------
    auto move_to_pose = [&](double x, double y, double z,
                            double qx, double qy, double qz, double qw,
                            const std::string& name) {
        RCLCPP_INFO(LOGGER, "Fahre zu: %s  (x=%.3f y=%.3f z=%.3f)", name.c_str(), x, y, z);
        geometry_msgs::msg::Pose target;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;
        target.orientation.x = qx;
        target.orientation.y = qy;
        target.orientation.z = qz;
        target.orientation.w = qw;

        move_group.setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group.execute(plan);
            RCLCPP_INFO(LOGGER, "Erfolgreich: %s", name.c_str());
        } else {
            RCLCPP_ERROR(LOGGER, "Planung fehlgeschlagen: %s", name.c_str());
        }
        move_group.clearPoseTargets();
    };

    // ----------------------------------------------------------------
    // Home-Pose als Joint-Target (definiert im SRDF)
    // ----------------------------------------------------------------
    std::vector<double> home = {0.0, 0.0, -1.5708, -1.5708, 0.0, 0.0};
    move_to_joints(home, "Home");

    // Aktuelle TCP-Pose ausgeben
    geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose();
    RCLCPP_INFO(LOGGER, "TCP @ Home: x=%.3f y=%.3f z=%.3f | qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
        current.pose.position.x, current.pose.position.y, current.pose.position.z,
        current.pose.orientation.x, current.pose.orientation.y,
        current.pose.orientation.z, current.pose.orientation.w);

    // ----------------------------------------------------------------
    // Kartesische Posen mit Quaternionen
    // Referenz: world Frame = Roboterbasis
    // Orientierung: Tool zeigt nach unten (x=1, y=0, z=0, w=0)
    // ----------------------------------------------------------------

    // Pose 1: vor dem Roboter, auf mittlerer Höhe
    move_to_pose(0.4, 0.0, 1.2,   1.0, 0.0, 0.0, 0.0,   "Pose 1 (vorne)");
    move_to_joints(home, "Home");

    // Pose 2: seitlich links, etwas tiefer
    move_to_pose(0.2, 0.4, 1.0,   1.0, 0.0, 0.0, 0.0,   "Pose 2 (links)");
    move_to_joints(home, "Home");

    // Pose 3: seitlich rechts
    move_to_pose(0.2, -0.4, 1.0,  1.0, 0.0, 0.0, 0.0,   "Pose 3 (rechts)");
    move_to_joints(home, "Home");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}
