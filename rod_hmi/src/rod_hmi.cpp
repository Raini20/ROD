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

struct SavedPose {
    std::string name;
    std::string robot;
    double x, y, z;
    double qx, qy, qz, qw;
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
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = mg->computeCartesianPath(waypoints, 0.005, trajectory);
    if (fraction > 0.8) {
        mg->execute(trajectory);
        set_status(robot_name + ": Bewegung OK");
    } else {
        set_status(robot_name + ": Nicht möglich (" + std::to_string((int)(fraction*100)) + "%)");
    }
}

void save_current_pose(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    const std::string& robot_name, const std::string& pose_name)
{
    auto p = mg->getCurrentPose().pose;
    SavedPose sp;
    sp.name = pose_name; sp.robot = robot_name;
    sp.x = p.position.x; sp.y = p.position.y; sp.z = p.position.z;
    sp.qx = p.orientation.x; sp.qy = p.orientation.y;
    sp.qz = p.orientation.z; sp.qw = p.orientation.w;
    std::lock_guard<std::mutex> lock(g_poses_mutex);
    g_saved_poses.push_back(sp);
    set_status("Pose gespeichert: " + pose_name);
}

void run_sequence()
{
    std::lock_guard<std::mutex> lock(g_poses_mutex);
    for (auto& sp : g_saved_poses) {
        auto mg = (sp.robot == "arm") ? g_arm_mg : g_scara_mg;
        if (!mg) continue;
        geometry_msgs::msg::Pose target;
        target.position.x = sp.x; target.position.y = sp.y; target.position.z = sp.z;
        target.orientation.x = sp.qx; target.orientation.y = sp.qy;
        target.orientation.z = sp.qz; target.orientation.w = sp.qw;
        mg->setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (mg->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            mg->execute(plan);
            set_status("Sequenz: " + sp.name + " OK");
        } else {
            set_status("Sequenz: " + sp.name + " FAILED");
        }
        mg->clearPoseTargets();
    }
    set_status("Sequenz abgeschlossen");
}

void export_poses(const std::string& path)
{
    std::ofstream f(path);
    f << "# ROD Pose Export\n# robot, name, x, y, z, qx, qy, qz, qw\n";
    std::lock_guard<std::mutex> lock(g_poses_mutex);
    for (auto& sp : g_saved_poses)
        f << sp.robot << ", " << sp.name << ", "
          << sp.x << ", " << sp.y << ", " << sp.z << ", "
          << sp.qx << ", " << sp.qy << ", " << sp.qz << ", " << sp.qw << "\n";
    set_status("Exportiert: " + path);
}

std::string read_file(const std::string& path) {
    std::ifstream f(path); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

std::string replace_all(std::string s, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.length(), to); pos += to.length();
    }
    return s;
}

void ros_thread_func(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    set_status("Starte Simulation (warte ~25s)...");

    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch rod_cell cell.launch.py > /tmp/cell.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(15));

    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch arm_moveit move_group.launch.py > /tmp/arm_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch scara_moveit move_group.launch.py > /tmp/scara_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    set_status("Verbinde mit Arm MoveGroup...");

    auto arm_pkg   = ament_index_cpp::get_package_share_directory("arm_moveit");
    auto robot_pkg = ament_index_cpp::get_package_share_directory("robot_arm_6dof_assembly");
    std::string urdf = replace_all(read_file(robot_pkg + "/urdf/robot_arm_6dof_assembly.urdf"),
                                   "$(find robot_arm_6dof_assembly)", robot_pkg);
    std::string srdf = read_file(arm_pkg + "/config/knickarm_6dof.srdf");

    rclcpp::NodeOptions opts_arm;
    opts_arm.automatically_declare_parameters_from_overrides(true);
    opts_arm.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("robot_description", urdf),
        rclcpp::Parameter("robot_description_semantic", srdf),
    });
    auto arm_node = rclcpp::Node::make_shared("hmi_arm_node", opts_arm);
    moveit::planning_interface::MoveGroupInterface::Options arm_opts("arm", "robot_description", "/arm");
    g_arm_mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_node, arm_opts);
    g_arm_mg->setMaxVelocityScalingFactor(0.1);
    g_arm_mg->setMaxAccelerationScalingFactor(0.1);
    g_arm_ready = true;
    set_status("Arm verbunden! Verbinde SCARA...");

    auto scara_pkg  = ament_index_cpp::get_package_share_directory("scara_moveit");
    auto scara4_pkg = ament_index_cpp::get_package_share_directory("scara_4");
    std::string surdf = replace_all(read_file(scara4_pkg + "/urdf/SCARA_4.urdf"),
                                    "$(find scara_4)", scara4_pkg);
    std::string ssrdf = read_file(scara_pkg + "/config/scara.srdf");

    rclcpp::NodeOptions opts_scara;
    opts_scara.automatically_declare_parameters_from_overrides(true);
    opts_scara.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true),
        rclcpp::Parameter("robot_description", surdf),
        rclcpp::Parameter("robot_description_semantic", ssrdf),
    });
    auto scara_node = rclcpp::Node::make_shared("hmi_scara_node", opts_scara);
    moveit::planning_interface::MoveGroupInterface::Options scara_opts("scara", "robot_description", "/scara");
    g_scara_mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(scara_node, scara_opts);
    g_scara_mg->setMaxVelocityScalingFactor(0.1);
    g_scara_mg->setMaxAccelerationScalingFactor(0.1);
    g_scara_ready = true;

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

    GLFWwindow* window = glfwCreateWindow(900, 680, "ROD HMI", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::GetIO().FontGlobalScale = 1.3f;

    int selected_robot = 0;
    float step_size = 0.05f;
    char pose_name_buf[64] = "Pose_1";
    static int pose_counter = 1;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0,0), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        ImGui::Begin("ROD HMI", nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|
            ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoScrollbar);

        ImGui::SetCursorPosX(300);
        ImGui::Text("ROD Robot Cell HMI");
        ImGui::Separator();

        {
            std::lock_guard<std::mutex> lock(g_status_mutex);
            ImGui::TextColored(ImVec4(0.3f,1.0f,0.3f,1.0f), "Status: %s", g_status.c_str());
        }

        ImGui::Text("Arm:");   ImGui::SameLine();
        if (g_arm_ready)   ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");
        ImGui::SameLine(); ImGui::Spacing(); ImGui::SameLine();
        ImGui::Text("SCARA:"); ImGui::SameLine();
        if (g_scara_ready) ImGui::TextColored(ImVec4(0,1,0,1), "VERBUNDEN");
        else               ImGui::TextColored(ImVec4(1,0.5f,0,1), "Warte...");

        ImGui::Separator();

        bool controls_enabled = g_ros_running.load();

        ImGui::Text("Roboter:"); ImGui::SameLine();
        ImGui::RadioButton("Arm",   &selected_robot, 0); ImGui::SameLine();
        ImGui::RadioButton("SCARA", &selected_robot, 1);

        auto get_mg = [&]() {
            return (selected_robot == 0) ? g_arm_mg : g_scara_mg;
        };
        auto robot_name = [&]() -> std::string {
            return (selected_robot == 0) ? "arm" : "scara";
        };

        ImGui::Separator();
        ImGui::Text("TCP Steuerung");
        ImGui::SetNextItemWidth(400);
        ImGui::SliderFloat("Schrittweite (m)", &step_size, 0.005f, 0.2f);

        float btn = 60.0f;
        float cx = 200.0f, cy = ImGui::GetCursorPosY() + 10;

        ImGui::BeginDisabled(!controls_enabled);

        ImGui::SetCursorPos(ImVec2(cx+btn, cy));
        if (ImGui::Button("Z+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),0,0,step_size,robot_name()); }).detach();

        ImGui::SetCursorPos(ImVec2(cx-btn, cy+btn+5));
        if (ImGui::Button("Y+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),0,step_size,0,robot_name()); }).detach();

        ImGui::SetCursorPos(ImVec2(cx, cy+btn+5));
        if (ImGui::Button("X+", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),step_size,0,0,robot_name()); }).detach();

        ImGui::SetCursorPos(ImVec2(cx+btn*2, cy+btn+5));
        if (ImGui::Button("X-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),-step_size,0,0,robot_name()); }).detach();

        ImGui::SetCursorPos(ImVec2(cx+btn*3, cy+btn+5));
        if (ImGui::Button("Y-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),0,-step_size,0,robot_name()); }).detach();

        ImGui::SetCursorPos(ImVec2(cx+btn, cy+(btn+5)*2));
        if (ImGui::Button("Z-", ImVec2(btn,btn)))
            std::thread([get_mg,&step_size,robot_name]{ move_cartesian(get_mg(),0,0,-step_size,robot_name()); }).detach();

        ImGui::SetCursorPosY(cy+(btn+5)*3+10);
        ImGui::EndDisabled();

        ImGui::Separator();

        ImGui::Text("Pose Name:"); ImGui::SameLine();
        ImGui::SetNextItemWidth(200);
        ImGui::InputText("##posename", pose_name_buf, sizeof(pose_name_buf));
        ImGui::SameLine();

        ImGui::BeginDisabled(!controls_enabled);
        if (ImGui::Button("Pose speichern")) {
            std::string pn(pose_name_buf), rn = robot_name();
            auto mg = get_mg();
            std::thread([mg,pn,rn]{ save_current_pose(mg,rn,pn); }).detach();
            pose_counter++;
            snprintf(pose_name_buf, sizeof(pose_name_buf), "Pose_%d", pose_counter);
        }
        ImGui::EndDisabled();

        ImGui::Separator();
        ImGui::Text("Gespeicherte Posen (%zu):", g_saved_poses.size());
        ImGui::BeginChild("poses_list", ImVec2(0,120), true);
        {
            std::lock_guard<std::mutex> lock(g_poses_mutex);
            for (int i = 0; i < (int)g_saved_poses.size(); i++) {
                auto& sp = g_saved_poses[i];
                ImGui::Text("[%d] [%s] %s  x=%.3f y=%.3f z=%.3f",
                    i+1, sp.robot.c_str(), sp.name.c_str(), sp.x, sp.y, sp.z);
            }
        }
        ImGui::EndChild();

        ImGui::BeginDisabled(!controls_enabled || g_saved_poses.empty());
        if (ImGui::Button("Sequenz ausführen"))
            std::thread(run_sequence).detach();
        ImGui::SameLine();
        if (ImGui::Button("Posen löschen")) {
            std::lock_guard<std::mutex> lock(g_poses_mutex);
            g_saved_poses.clear(); pose_counter = 1;
            snprintf(pose_name_buf, sizeof(pose_name_buf), "Pose_1");
            set_status("Posen gelöscht");
        }
        ImGui::SameLine();
        if (ImGui::Button("Exportieren"))
            std::thread([]{ export_poses("/tmp/rod_poses.csv"); }).detach();
        ImGui::EndDisabled();

        ImGui::Separator();
        if (controls_enabled) {
            auto mg = get_mg();
            if (mg) {
                auto p = mg->getCurrentPose().pose;
                ImGui::Text("TCP [%s]: x=%.3f  y=%.3f  z=%.3f",
                    robot_name().c_str(), p.position.x, p.position.y, p.position.z);
            }
        }

        ImGui::End();
        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
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
