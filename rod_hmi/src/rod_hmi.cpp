#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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

static std::atomic<bool> g_arm_ready{false};
static std::atomic<bool> g_scara_ready{false};
static std::atomic<bool> g_ros_running{false};
static std::string g_status = "Starte ROS...";
static std::mutex g_status_mutex;
static std::vector<SavedPose> g_saved_poses;
static std::mutex g_poses_mutex;
static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_arm_mg;
static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_scara_mg;

void set_status(const std::string& s) {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    g_status = s;
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "%s", s.c_str());
}

void execute_action(Action a, const std::string& robot) {
    switch(a) {
        case Action::Pick:
            set_status(robot + ": Pick — Platzhalter (link_attacher)");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        case Action::Place:
            set_status(robot + ": Place — Platzhalter (link_attacher)");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        case Action::Screw:
            set_status(robot + ": Screw — Platzhalter");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        case Action::Unscrew:
            set_status(robot + ": Unscrew — Platzhalter");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
        default: break;
    }
}

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
            RCLCPP_WARN(rclcpp::get_logger("rod_hmi"), "Kartesisch %.0f%% — Fallback setPoseTarget", fraction*100.0);
            mg->setPoseTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                mg->execute(plan);
                RCLCPP_INFO(rclcpp::get_logger("rod_hmi"), "Sequenz [%zu/%zu]: %s OK (Fallback)", i+1, poses_copy.size(), sp.name.c_str());
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rod_hmi"), "Sequenz [%zu/%zu]: %s FAILED", i+1, poses_copy.size(), sp.name.c_str());
                set_status("FEHLER: " + sp.name); mg->clearPoseTargets(); continue;
            }
            mg->clearPoseTargets();
        }
        execute_action(sp.action, sp.robot);
    }
    set_status("Sequenz abgeschlossen (" + std::to_string(poses_copy.size()) + " Posen)");
}

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

std::string read_file(const std::string& path) {
    std::ifstream f(path); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}
std::string replace_all(std::string s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) { s.replace(pos, from.length(), to); pos += to.length(); }
    return s;
}

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

int main(int argc, char** argv)
{
    std::thread ros_t(ros_thread_func, argc, argv);
    ros_t.detach();

    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(960, 750, "ROD HMI", nullptr, nullptr);
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
        ImGui::Text("Arm:");   ImGui::SameLine();
        if (g_arm_ready)   ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");
        ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
        ImGui::Text("SCARA:"); ImGui::SameLine();
        if (g_scara_ready) ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");
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
        ImGui::BeginChild("poses_list", ImVec2(0,110), true);
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

        ImGui::Separator();
        { std::lock_guard<std::mutex> lock(tcp_mutex);
          ImGui::Text("TCP [%s]:  x = %7.3f   y = %7.3f   z = %7.3f",
              rname().c_str(), tcp_x, tcp_y, tcp_z); }

        // ---- Gelenkswinkel Slider ----
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
                        g_arm_mg->execute(p);
                        set_status("arm: Joints angefahren");
                    } else set_status("arm: Joints Planung fehlgeschlagen");
                }).detach();
            }
        } else {
            const char* scara_names[] = {"J1","J2","J3 (lin)","J4"};
            float limits_lo[] = {-3.14159f, -3.14159f, -0.4f, -3.14159f};
            float limits_hi[] = { 3.14159f,  3.14159f,  0.0f,  3.14159f};
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
                        g_scara_mg->execute(p);
                        set_status("scara: Joints angefahren");
                    } else set_status("scara: Joints Planung fehlgeschlagen");
                }).detach();
            }
        }
        ImGui::EndDisabled();
        
        ImGui::End();
        ImGui::Render();
        int w, h; glfwGetFramebufferSize(window, &w, &h);
        glViewport(0,0,w,h);
        glClearColor(0.08f,0.08f,0.10f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

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
