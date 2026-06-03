#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <mutex>
#include <map>
#include <cmath>

// -----------------------------------------------------------------------
// Aktionen
// -----------------------------------------------------------------------
enum class Action { None, Pick, Place, Screw, Unscrew };

std::string action_to_string(Action a) {
    switch(a) {
        case Action::Pick:    return "Pick";
        case Action::Place:   return "Place";
        case Action::Screw:   return "Screw";
        case Action::Unscrew: return "Unscrew";
        default:              return "None";
    }
}

struct SavedPose {
    std::string name, robot;
    double x, y, z, qx, qy, qz, qw;
    Action action = Action::None;
};

// -----------------------------------------------------------------------
// Globale Variablen
// -----------------------------------------------------------------------
static std::atomic<bool> g_arm_ready{false};
static std::atomic<bool> g_scara_ready{false};
static std::atomic<bool> g_ros_running{false};
static std::string g_status = "Starte ROS...";
static std::mutex g_status_mutex;
static std::vector<SavedPose> g_saved_poses;
static std::mutex g_poses_mutex;

static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_arm_mg;
static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_scara_mg;

// Pick/Place State
static gz::transport::Node g_gz_node;
static std::string g_picked_object = "";
static std::atomic<bool> g_picking{false};
static double g_off_x=0, g_off_y=0, g_off_z=0;
static std::mutex g_obj_mutex;
static std::map<std::string, std::array<double,3>> g_object_positions = {
    {"toaster_shell", {-0.75, 0.45, 1.0}},
    {"toaster_innen", {0.0,   0.45, 1.168}},
};

void set_status(const std::string& s) {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    g_status = s;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "%s", s.c_str());
}

// -----------------------------------------------------------------------
// Gz Transport: Pose setzen
// -----------------------------------------------------------------------
void gz_set_pose(const std::string& name, double x, double y, double z,
                 double qx=0, double qy=0, double qz=0, double qw=1)
{
    gz::msgs::Pose req;
    gz::msgs::Boolean rep;
    bool result;
    req.set_name(name);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_x(qx);
    req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz);
    req.mutable_orientation()->set_w(qw);
    g_gz_node.Request("/world/empty/set_pose", req, 200, rep, result);
}

// -----------------------------------------------------------------------
// Pick
// -----------------------------------------------------------------------
void do_pick(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
{
    auto tcp = mg->getCurrentPose().pose;
    // Arm spawn offset: Arm steht in Gazebo bei (-0.75, 0, 1.0)
    const double ARM_OX=-0.75, ARM_OY=0.0, ARM_OZ=1.0;
    double tx = tcp.position.x + ARM_OX;
    double ty = tcp.position.y + ARM_OY;
    double tz = tcp.position.z + ARM_OZ;

    // Nächstes Objekt finden
    std::string closest = "";
    double min_dist = 1e9;
    {
        std::lock_guard<std::mutex> lock(g_obj_mutex);
        for (auto& [name, pos] : g_object_positions) {
            double d = std::sqrt(std::pow(pos[0]-tx,2)+std::pow(pos[1]-ty,2)+std::pow(pos[2]-tz,2));
            if (d < min_dist) { min_dist = d; closest = name; }
        }
    }

    if (closest.empty()) { set_status("Kein Objekt gefunden!"); return; }

    {
        std::lock_guard<std::mutex> lock(g_obj_mutex);
        auto& pos = g_object_positions[closest];
        g_off_x = pos[0]-tx; g_off_y = pos[1]-ty; g_off_z = pos[2]-tz;
    }

    g_picked_object = closest;
    g_picking = true;

    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
        "Pick: %s  Distanz=%.3fm  Offset=(%.3f, %.3f, %.3f)",
        closest.c_str(), min_dist, g_off_x, g_off_y, g_off_z);
    set_status("Pick: " + closest + " (Dist: " + std::to_string((int)(min_dist*100)) + "cm)");

    // Follow-Thread
    std::thread([mg](){
        while (g_picking && mg) {
            auto tcp = mg->getCurrentPose().pose;
            const double ARM_OX=-0.75, ARM_OZ=1.0;
            double nx = tcp.position.x + ARM_OX + g_off_x;
            double ny = tcp.position.y + g_off_y;
            double nz = tcp.position.z + ARM_OZ + g_off_z;
            gz_set_pose(g_picked_object, nx, ny, nz,
                tcp.orientation.x, tcp.orientation.y,
                tcp.orientation.z, tcp.orientation.w);
            {
                std::lock_guard<std::mutex> lock(g_obj_mutex);
                g_object_positions[g_picked_object] = {nx, ny, nz};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
    }).detach();
}

// -----------------------------------------------------------------------
// Place
// -----------------------------------------------------------------------
void do_place()
{
    g_picking = false;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Place: %s abgesetzt", g_picked_object.c_str());
    set_status("Place: " + g_picked_object + " abgesetzt");
    g_picked_object = "";
}

// -----------------------------------------------------------------------
// Aktion ausführen
// -----------------------------------------------------------------------
void execute_action(Action a, const std::string& robot,
                    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
{
    switch(a) {
        case Action::Pick:
            if (mg) do_pick(mg);
            break;
        case Action::Place:
            do_place();
            break;
        case Action::Screw:
            set_status(robot + ": Screw — Platzhalter");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        case Action::Unscrew:
            set_status(robot + ": Unscrew — Platzhalter");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        default: break;
    }
}

// -----------------------------------------------------------------------
// Kartesische Bewegung
// -----------------------------------------------------------------------
void move_cartesian(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    double dx, double dy, double dz, const std::string& robot_name)
{
    auto current = mg->getCurrentPose().pose;
    geometry_msgs::msg::Pose target = current;
    target.position.x += dx;
    target.position.y += dy;
    target.position.z += dz;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
        "[%s] TCP → x=%.3f y=%.3f z=%.3f",
        robot_name.c_str(), target.position.x, target.position.y, target.position.z);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = mg->computeCartesianPath(waypoints, 0.005, trajectory);
    if (fraction > 0.8) {
        mg->execute(trajectory);
        set_status(robot_name + ": Bewegung OK (" + std::to_string((int)(fraction*100)) + "%)");
    } else {
        set_status(robot_name + ": Nicht möglich (" + std::to_string((int)(fraction*100)) + "%)");
    }
}

// -----------------------------------------------------------------------
// Rotation
// -----------------------------------------------------------------------
void move_rotation(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    double droll, double dpitch, double dyaw, const std::string& robot_name)
{
    auto current = mg->getCurrentPose().pose;
    geometry_msgs::msg::Pose target = current;
    double qx=current.orientation.x, qy=current.orientation.y;
    double qz=current.orientation.z, qw=current.orientation.w;
    double cr=cos(droll/2), sr=sin(droll/2);
    double cp=cos(dpitch/2), sp=sin(dpitch/2);
    double cy=cos(dyaw/2), sy=sin(dyaw/2);
    double dqw=cr*cp*cy+sr*sp*sy, dqx=sr*cp*cy-cr*sp*sy;
    double dqy=cr*sp*cy+sr*cp*sy, dqz=cr*cp*sy-sr*sp*cy;
    target.orientation.w=qw*dqw-qx*dqx-qy*dqy-qz*dqz;
    target.orientation.x=qw*dqx+qx*dqw+qy*dqz-qz*dqy;
    target.orientation.y=qw*dqy-qx*dqz+qy*dqw+qz*dqx;
    target.orientation.z=qw*dqz+qx*dqy-qy*dqx+qz*dqw;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
        "[%s] Rotation dR=%.2f dP=%.2f dY=%.2f", robot_name.c_str(), droll, dpitch, dyaw);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = mg->computeCartesianPath(waypoints, 0.005, trajectory);
    if (fraction > 0.8) { mg->execute(trajectory); set_status(robot_name + ": Rotation OK"); }
    else { set_status(robot_name + ": Rotation nicht möglich"); }
}

// -----------------------------------------------------------------------
// Pose speichern
// -----------------------------------------------------------------------
void save_current_pose(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    const std::string& robot_name, const std::string& pose_name, Action action)
{
    auto p = mg->getCurrentPose().pose;
    SavedPose sp;
    sp.name=pose_name; sp.robot=robot_name; sp.action=action;
    sp.x=p.position.x; sp.y=p.position.y; sp.z=p.position.z;
    sp.qx=p.orientation.x; sp.qy=p.orientation.y;
    sp.qz=p.orientation.z; sp.qw=p.orientation.w;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
        "Pose gespeichert: [%s] %s  x=%.3f y=%.3f z=%.3f  Aktion=%s",
        robot_name.c_str(), pose_name.c_str(), sp.x, sp.y, sp.z,
        action_to_string(action).c_str());
    std::lock_guard<std::mutex> lock(g_poses_mutex);
    g_saved_poses.push_back(sp);
    set_status("Pose gespeichert: " + pose_name + " [" + action_to_string(action) + "]");
}

// -----------------------------------------------------------------------
// Sequenz
// -----------------------------------------------------------------------
void run_sequence()
{
    std::vector<SavedPose> poses_copy;
    { std::lock_guard<std::mutex> lock(g_poses_mutex); poses_copy = g_saved_poses; }
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Starte Sequenz mit %zu Posen", poses_copy.size());
    set_status("Sequenz gestartet...");
    for (size_t i = 0; i < poses_copy.size(); i++) {
        auto& sp = poses_copy[i];
        auto mg = (sp.robot == "arm") ? g_arm_mg : g_scara_mg;
        if (!mg) continue;
        RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
            "Sequenz [%zu/%zu]: %s [%s] x=%.3f y=%.3f z=%.3f",
            i+1, poses_copy.size(), sp.name.c_str(), sp.robot.c_str(), sp.x, sp.y, sp.z);
        set_status("Sequenz " + std::to_string(i+1) + "/" +
                   std::to_string(poses_copy.size()) + ": " + sp.name);
        geometry_msgs::msg::Pose target;
        target.position.x=sp.x; target.position.y=sp.y; target.position.z=sp.z;
        target.orientation.x=sp.qx; target.orientation.y=sp.qy;
        target.orientation.z=sp.qz; target.orientation.w=sp.qw;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = mg->computeCartesianPath(waypoints, 0.005, trajectory);
        if (fraction > 0.5) {
            mg->execute(trajectory);
            RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),
                "Sequenz [%zu/%zu]: %s OK (%.0f%%)", i+1, poses_copy.size(), sp.name.c_str(), fraction*100.0);
        } else {
            mg->setPoseTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                mg->execute(plan);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rod_hmi"), "Sequenz [%zu/%zu]: %s FAILED", i+1, poses_copy.size(), sp.name.c_str());
                mg->clearPoseTargets(); continue;
            }
            mg->clearPoseTargets();
        }
        execute_action(sp.action, sp.robot, mg);
    }
    set_status("Sequenz abgeschlossen (" + std::to_string(poses_copy.size()) + " Posen)");
}

// -----------------------------------------------------------------------
// CSV Export
// -----------------------------------------------------------------------
void export_poses(const std::string& path) {
    std::ofstream f(path);
    f << "# ROD Pose Export\n# robot, name, x, y, z, qx, qy, qz, qw, action\n";
    std::lock_guard<std::mutex> lock(g_poses_mutex);
    for (auto& sp : g_saved_poses)
        f << sp.robot << ", " << sp.name << ", "
          << sp.x << ", " << sp.y << ", " << sp.z << ", "
          << sp.qx << ", " << sp.qy << ", " << sp.qz << ", " << sp.qw
          << ", " << action_to_string(sp.action) << "\n";
    set_status("Exportiert: " + path);
}

// -----------------------------------------------------------------------
// Hilfsfunktionen
// -----------------------------------------------------------------------
std::string read_file(const std::string& path) {
    std::ifstream f(path); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
std::string replace_all(std::string s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) { s.replace(pos, from.length(), to); pos += to.length(); }
    return s;
}

// -----------------------------------------------------------------------
// ROS Thread
// -----------------------------------------------------------------------
void ros_thread_func(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "ROS2 initialisiert");
    set_status("Starte Simulation (warte ~25s)...");
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch rod_cell cell.launch.py > /tmp/cell.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(15));
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Gazebo gestartet");
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch arm_moveit move_group.launch.py > /tmp/arm_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Arm MoveGroup gestartet");
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch scara_moveit move_group.launch.py > /tmp/scara_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "SCARA MoveGroup gestartet");
    set_status("Verbinde mit Arm MoveGroup...");

    auto arm_pkg   = ament_index_cpp::get_package_share_directory("arm_moveit");
    auto robot_pkg = ament_index_cpp::get_package_share_directory("robot_arm_6dof_assembly");
    std::string urdf = replace_all(read_file(robot_pkg + "/urdf/robot_arm_6dof_assembly.urdf"),
                                   "$(find robot_arm_6dof_assembly)", robot_pkg);
    std::string srdf = read_file(arm_pkg + "/config/knickarm_6dof.srdf");
    rclcpp::NodeOptions opts_arm;
    opts_arm.automatically_declare_parameters_from_overrides(true);
    opts_arm.arguments({"--ros-args", "-r", "joint_states:=/arm/joint_states"});
    opts_arm.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("robot_description", urdf),
        rclcpp::Parameter("robot_description_semantic", srdf)});
    auto arm_node = rclcpp::Node::make_shared("hmi_arm_node", "/arm_hmi", opts_arm);
    moveit::planning_interface::MoveGroupInterface::Options arm_opts("arm", "robot_description", "/arm");
    g_arm_mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_node, arm_opts);
    g_arm_mg->setMaxVelocityScalingFactor(0.1);
    g_arm_mg->setMaxAccelerationScalingFactor(0.1);
    g_arm_ready = true;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Arm verbunden");
    set_status("Arm verbunden! Verbinde SCARA...");

    auto scara_pkg  = ament_index_cpp::get_package_share_directory("scara_moveit");
    auto scara4_pkg = ament_index_cpp::get_package_share_directory("scara_4");
    std::string surdf = replace_all(read_file(scara4_pkg + "/urdf/SCARA_4.urdf"), "$(find scara_4)", scara4_pkg);
    std::string ssrdf = read_file(scara_pkg + "/config/scara.srdf");
    rclcpp::NodeOptions opts_scara;
    opts_scara.automatically_declare_parameters_from_overrides(true);
    opts_scara.arguments({"--ros-args", "-r", "joint_states:=/scara/joint_states"});
    opts_scara.parameter_overrides({rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("scara_description", surdf),
        rclcpp::Parameter("scara_description_semantic", ssrdf)});
    auto scara_node = rclcpp::Node::make_shared("hmi_scara_node", "/scara_hmi", opts_scara);
    moveit::planning_interface::MoveGroupInterface::Options scara_opts("scara", "scara_description", "/scara");
    g_scara_mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(scara_node, scara_opts);
    g_scara_mg->setMaxVelocityScalingFactor(0.1);
    g_scara_mg->setMaxAccelerationScalingFactor(0.1);
    g_scara_ready = true;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "SCARA verbunden");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_node);
    executor.add_node(scara_node);
    g_ros_running = true;
    set_status("Bereit!");
    executor.spin();
    rclcpp::shutdown();
}

// -----------------------------------------------------------------------
// Main / GUI Loop
// -----------------------------------------------------------------------
int main(int argc, char** argv)
{
    std::thread ros_t(ros_thread_func, argc, argv);
    ros_t.detach();

    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(960, 800, "ROD HMI", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::GetIO().FontGlobalScale = 1.2f;

    int selected_robot = 0;
    float step_size = 0.05f, rot_step = 0.1f;
    char pose_name_buf[64] = "Pose_1";
    int pose_counter = 1, selected_action = 0;
    float arm_joints[6]   = {0.0f, 0.0f, -1.5708f, -1.5708f, 0.0f, 0.0f};
    float scara_joints[4] = {0.0f, 0.0f,  0.0f,     0.0f};

    static double tcp_x=0, tcp_y=0, tcp_z=0;
    static std::mutex tcp_mutex;
    std::thread tcp_updater([&](){
        while (!glfwWindowShouldClose(window)) {
            if (g_ros_running) {
                try {
                    auto mg = (selected_robot == 0) ? g_arm_mg : g_scara_mg;
                    if (mg) { auto p = mg->getCurrentPose().pose;
                        std::lock_guard<std::mutex> lock(tcp_mutex);
                        tcp_x=p.position.x; tcp_y=p.position.y; tcp_z=p.position.z; }
                } catch(...) {}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
    tcp_updater.detach();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        ImGui::Begin("ROD HMI", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

        ImGui::SetCursorPosX(350);
        ImGui::TextColored(ImVec4(0.4f,0.8f,1.0f,1.0f), "ROD Robot Cell HMI");
        ImGui::Separator();
        { std::lock_guard<std::mutex> lock(g_status_mutex);
          ImGui::TextColored(ImVec4(0.3f,1.0f,0.3f,1.0f), "Status: %s", g_status.c_str()); }

        // Verbindungsstatus + Pick-Status
        ImGui::Text("Arm:");   ImGui::SameLine();
        if (g_arm_ready)   ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");
        ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
        ImGui::Text("SCARA:"); ImGui::SameLine();
        if (g_scara_ready) ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");
        ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
        if (g_picking) {
            ImGui::TextColored(ImVec4(1,1,0,1), "GREIFT: %s", g_picked_object.c_str());
        }
        ImGui::Separator();

        bool enabled = g_ros_running.load();
        ImGui::Text("Roboter:"); ImGui::SameLine();
        ImGui::RadioButton("Arm", &selected_robot, 0); ImGui::SameLine();
        ImGui::RadioButton("SCARA", &selected_robot, 1);

        auto get_mg = [&](){ return (selected_robot==0) ? g_arm_mg : g_scara_mg; };
        auto rname  = [&]() -> std::string { return (selected_robot==0) ? "arm" : "scara"; };

        ImGui::Separator();
        ImGui::Text("TCP Translation");
        ImGui::SetNextItemWidth(350);
        ImGui::SliderFloat("Schrittweite m", &step_size, 0.005f, 0.2f);

        float btn=55.0f, cx=180.0f, cy=ImGui::GetCursorPosY()+8;
        ImGui::BeginDisabled(!enabled);
        ImGui::SetCursorPos(ImVec2(cx+btn, cy));
        if (ImGui::Button("Z+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),0,0,step_size,rname()); }).detach();
        ImGui::SetCursorPos(ImVec2(cx-btn, cy+btn+4));
        if (ImGui::Button("Y+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),0,step_size,0,rname()); }).detach();
        ImGui::SetCursorPos(ImVec2(cx, cy+btn+4));
        if (ImGui::Button("X+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),step_size,0,0,rname()); }).detach();
        ImGui::SetCursorPos(ImVec2(cx+btn*2, cy+btn+4));
        if (ImGui::Button("X-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),-step_size,0,0,rname()); }).detach();
        ImGui::SetCursorPos(ImVec2(cx+btn*3, cy+btn+4));
        if (ImGui::Button("Y-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),0,-step_size,0,rname()); }).detach();
        ImGui::SetCursorPos(ImVec2(cx+btn, cy+(btn+4)*2));
        if (ImGui::Button("Z-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,rname]{ move_cartesian(get_mg(),0,0,-step_size,rname()); }).detach();
        ImGui::SetCursorPosY(cy+(btn+4)*3+8);

        if (selected_robot == 0) {
            ImGui::Separator();
            ImGui::Text("TCP Rotation (nur Arm)");
            ImGui::SetNextItemWidth(350);
            ImGui::SliderFloat("Rot. Schritt rad", &rot_step, 0.02f, 0.5f);
            float rcx=180.0f, rcy=ImGui::GetCursorPosY()+8;
            ImGui::SetCursorPos(ImVec2(rcx, rcy));
            if (ImGui::Button("R+", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),rot_step,0,0,rname()); }).detach();
            ImGui::SetCursorPos(ImVec2(rcx+btn+4, rcy));
            if (ImGui::Button("R-", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),-rot_step,0,0,rname()); }).detach();
            ImGui::SetCursorPos(ImVec2(rcx+btn*2+8, rcy));
            if (ImGui::Button("P+", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),0,rot_step,0,rname()); }).detach();
            ImGui::SetCursorPos(ImVec2(rcx+btn*3+12, rcy));
            if (ImGui::Button("P-", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),0,-rot_step,0,rname()); }).detach();
            ImGui::SetCursorPos(ImVec2(rcx+btn*4+16, rcy));
            if (ImGui::Button("Y+##rot", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),0,0,rot_step,rname()); }).detach();
            ImGui::SetCursorPos(ImVec2(rcx+btn*5+20, rcy));
            if (ImGui::Button("Y-##rot", ImVec2(btn,btn)))
                std::thread([get_mg,&rot_step,rname]{ move_rotation(get_mg(),0,0,-rot_step,rname()); }).detach();
            ImGui::SetCursorPosY(rcy+btn+12);
            ImGui::TextDisabled("R=Roll  P=Pitch  Y=Yaw");
        }
        ImGui::EndDisabled();
        ImGui::Separator();

        ImGui::Text("Pose:"); ImGui::SameLine();
        ImGui::SetNextItemWidth(150);
        ImGui::InputText("##posename", pose_name_buf, sizeof(pose_name_buf));
        ImGui::SameLine();
        ImGui::Text("Aktion:"); ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (selected_robot == 0) {
            const char* arm_actions[] = {"None","Pick","Place"};
            if (selected_action>2) selected_action=0;
            ImGui::Combo("##action", &selected_action, arm_actions, 3);
        } else {
            const char* scara_actions[] = {"None","Screw","Unscrew"};
            if (selected_action>2) selected_action=0;
            ImGui::Combo("##action", &selected_action, scara_actions, 3);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!enabled);
        if (ImGui::Button("Speichern")) {
            Action act = Action::None;
            if (selected_robot==0) { if(selected_action==1) act=Action::Pick; else if(selected_action==2) act=Action::Place; }
            else                   { if(selected_action==1) act=Action::Screw; else if(selected_action==2) act=Action::Unscrew; }
            std::string pn(pose_name_buf), rn=rname(); auto mg=get_mg();
            std::thread([mg,pn,rn,act]{ save_current_pose(mg,rn,pn,act); }).detach();
            pose_counter++; snprintf(pose_name_buf,sizeof(pose_name_buf),"Pose_%d",pose_counter);
            selected_action=0;
        }
        ImGui::EndDisabled();
        ImGui::Separator();

        ImGui::Text("Gespeicherte Posen (%zu):", g_saved_poses.size());
        ImGui::BeginChild("poses_list", ImVec2(0,100), true);
        { std::lock_guard<std::mutex> lock(g_poses_mutex);
          for (int i=0; i<(int)g_saved_poses.size(); i++) {
              auto& sp=g_saved_poses[i];
              ImGui::Text("[%d] [%s] %-12s  x=%6.3f y=%6.3f z=%6.3f  → %s",
                  i+1, sp.robot.c_str(), sp.name.c_str(), sp.x, sp.y, sp.z,
                  action_to_string(sp.action).c_str()); } }
        ImGui::EndChild();

        ImGui::BeginDisabled(!enabled || g_saved_poses.empty());
        if (ImGui::Button("Sequenz ausführen")) std::thread(run_sequence).detach();
        ImGui::SameLine();
        if (ImGui::Button("Posen löschen")) {
            std::lock_guard<std::mutex> lock(g_poses_mutex); g_saved_poses.clear();
            pose_counter=1; snprintf(pose_name_buf,sizeof(pose_name_buf),"Pose_1");
            set_status("Posen gelöscht"); }
        ImGui::SameLine();
        if (ImGui::Button("Exportieren")) std::thread([]{ export_poses("/tmp/rod_poses.csv"); }).detach();
        ImGui::EndDisabled();

        // Objekte Status
        ImGui::Separator();
        ImGui::Text("Objekte in Simulation:");
        { std::lock_guard<std::mutex> lock(g_obj_mutex);
          for (auto& [name, pos] : g_object_positions) {
              bool is_picked = (g_picking && g_picked_object == name);
              if (is_picked) ImGui::TextColored(ImVec4(1,1,0,1), "  [GEGRIFFEN] %s  x=%.3f y=%.3f z=%.3f", name.c_str(), pos[0], pos[1], pos[2]);
              else           ImGui::Text("  %s  x=%.3f y=%.3f z=%.3f", name.c_str(), pos[0], pos[1], pos[2]);
          }
        }

        ImGui::Separator();
        ImGui::Text("Gelenkswinkel (absolut)");
        ImGui::BeginDisabled(!enabled);
        if (selected_robot == 0) {
            const char* arm_names[] = {"J1","J2","J3","J4","J5","J6"};
            for (int i = 0; i < 6; i++) {
                ImGui::SetNextItemWidth(300);
                ImGui::SliderFloat(arm_names[i], &arm_joints[i], -3.14159f, 3.14159f);
            }
            if (ImGui::Button("Arm: Joints anfahren")) {
                std::vector<double> jv(arm_joints, arm_joints+6);
                std::thread([jv](){
                    g_arm_mg->setJointValueTarget(jv);
                    moveit::planning_interface::MoveGroupInterface::Plan p;
                    if (g_arm_mg->plan(p) == moveit::core::MoveItErrorCode::SUCCESS) {
                        g_arm_mg->execute(p); set_status("arm: Joints angefahren");
                    } else set_status("arm: Joints Planung fehlgeschlagen");
                }).detach();
            }
        } else {
            const char* scara_names[] = {"J1","J2","J3 (lin)","J4"};
            float limits_lo[] = {-3.14159f,-3.14159f,-0.4f,-3.14159f};
            float limits_hi[] = { 3.14159f, 3.14159f, 0.0f, 3.14159f};
            for (int i = 0; i < 4; i++) {
                ImGui::SetNextItemWidth(300);
                ImGui::SliderFloat(scara_names[i], &scara_joints[i], limits_lo[i], limits_hi[i]);
            }
            if (ImGui::Button("SCARA: Joints anfahren")) {
                std::vector<double> jv(scara_joints, scara_joints+4);
                std::thread([jv](){
                    g_scara_mg->setJointValueTarget(jv);
                    moveit::planning_interface::MoveGroupInterface::Plan p;
                    if (g_scara_mg->plan(p) == moveit::core::MoveItErrorCode::SUCCESS) {
                        g_scara_mg->execute(p); set_status("scara: Joints angefahren");
                    } else set_status("scara: Joints Planung fehlgeschlagen");
                }).detach();
            }
        }
        ImGui::EndDisabled();

        ImGui::Separator();
        { std::lock_guard<std::mutex> lock(tcp_mutex);
          ImGui::Text("TCP [%s]:  x = %7.3f   y = %7.3f   z = %7.3f",
              rname().c_str(), tcp_x, tcp_y, tcp_z); }

        ImGui::End();
        ImGui::Render();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0,0,w,h);
        glClearColor(0.08f,0.08f,0.10f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    g_picking = false;
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    std::system("pkill -f 'ros2 launch rod_cell'");
    std::system("pkill -f 'move_group'");
    std::system("pkill -f 'gz sim'");
    return 0;
}
