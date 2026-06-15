// =============================================================================
// rod_hmi.cpp  —  ROD Robot Cell HMI
// FH Technikum Wien | Digital Manufacturing, Automation & Robotics
// =============================================================================

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
#include <deque>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <map>
#include <cmath>

// =============================================================================
// DATA TYPES
// =============================================================================

enum class Action { None, Pick, Place, Screw, Unscrew, SceneReset };

static const char* action_str(Action a) {
    switch(a){
        case Action::Pick:       return "Pick";
        case Action::Place:      return "Place";
        case Action::Screw:      return "Screw";
        case Action::Unscrew:    return "Unscrew";
        case Action::SceneReset: return "Reset";
        default:                 return "None";
    }
}
static Action action_from_str(const std::string& s) {
    if(s=="Pick")    return Action::Pick;
    if(s=="Place")   return Action::Place;
    if(s=="Screw")   return Action::Screw;
    if(s=="Unscrew") return Action::Unscrew;
    if(s=="Reset")   return Action::SceneReset;
    return Action::None;
}

struct SavedPose {
    std::string name, robot;
    double x{},y{},z{},qx{},qy{},qz{},qw{1};
    Action action = Action::None;
    std::vector<double> joints;
};

struct PickedObj {
    std::string name;
    double off_x{},off_y{},off_z{};
    double off_rqx{},off_rqy{},off_rqz{},off_rqw{1};
};

// =============================================================================
// GLOBAL STATE
// =============================================================================

static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_arm_mg;
static std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_scara_mg;
static std::atomic<bool> g_arm_ready{false};
static std::atomic<bool> g_scara_ready{false};
static std::atomic<bool> g_ros_running{false};

static std::string             g_status = "Starte ROS...";
static std::mutex              g_status_mx;
static std::deque<std::string> g_log;
static std::mutex              g_log_mx;
static const int               LOG_MAX = 40;

static std::vector<SavedPose> g_poses;
static std::mutex             g_poses_mx;
static int                    g_sel = 0;

static float      g_jv_arm[6]   = {};
static float      g_jv_scara[4] = {};
static std::mutex g_jv_mx;

// World-frame initial positions/orientations matching cell.launch.py exactly
static const std::map<std::string,std::array<double,3>> k_init_pos = {
    {"toaster_shell",{-0.75, 0.45, 1.000}},
    {"toaster_innen",{ 0.00, 0.45, 1.168}},
    {"schraube_1",   { 0.090,0.505,1.168}},
    {"schraube_2",   {-0.090,0.505,1.168}},
    {"schraube_3",   { 0.090,0.395,1.168}},
    {"schraube_4",   {-0.090,0.395,1.168}},
};
static const std::map<std::string,std::array<double,4>> k_init_ori = {
    {"toaster_shell",{0.0,0.0,0.0,1.0}},
    {"toaster_innen",{1.0,0.0,0.0,0.0}},
    {"schraube_1",   {1.0,0.0,0.0,0.0}},
    {"schraube_2",   {1.0,0.0,0.0,0.0}},
    {"schraube_3",   {1.0,0.0,0.0,0.0}},
    {"schraube_4",   {1.0,0.0,0.0,0.0}},
};
static gz::transport::Node                         g_gz;
static std::atomic<bool>                           g_picking{false};
static std::mutex                                  g_obj_mx;
static std::map<std::string,std::array<double,3>>  g_obj_pos = k_init_pos;
static std::map<std::string,std::array<double,4>>  g_obj_ori = k_init_ori;
static std::vector<PickedObj>                      g_picked;

// Robot world-frame base offsets (from cell.launch.py -x -y -z args)
static constexpr double ARM_OX=-0.75, ARM_OY=0.0, ARM_OZ=1.0;
static constexpr double SCARA_OX=1.00, SCARA_OY=0.0, SCARA_OZ=1.0;

// Motion/sequence control
static std::atomic<bool>  g_estop{false};
static std::atomic<bool>  g_seq_running{false};
static std::atomic<bool>  g_seq_paused{false};
static std::atomic<bool>  g_seq_stop_req{false};
static std::atomic<int>   g_seq_step{-1};
static std::atomic<int>   g_seq_count{0};
static std::atomic<bool>  g_seq_loop{false};
static float              g_vel_pct = 30.0f;

// =============================================================================
// UTILITIES
// =============================================================================

static void set_status(const std::string& s) {
    {std::lock_guard<std::mutex> lk(g_status_mx); g_status=s;}
    {
        std::lock_guard<std::mutex> lk(g_log_mx);
        std::time_t t=std::time(nullptr); char ts[10];
        std::strftime(ts,sizeof(ts),"%H:%M:%S",std::localtime(&t));
        g_log.push_back(std::string(ts)+"  "+s);
        if((int)g_log.size()>LOG_MAX) g_log.pop_front();
    }
    RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),"%s",s.c_str());
}

static std::string read_file(const std::string& p)
{ std::ifstream f(p); std::stringstream ss; ss<<f.rdbuf(); return ss.str(); }

static std::string replace_all(std::string s,const std::string& f,const std::string& t) {
    for(size_t p=0;(p=s.find(f,p))!=std::string::npos;p+=t.size()) s.replace(p,f.size(),t);
    return s;
}

// =============================================================================
// GZ TRANSPORT
// =============================================================================

static void gz_set_pose(const std::string& name,double x,double y,double z,
                        double qx=0,double qy=0,double qz=0,double qw=1) {
    gz::msgs::Pose req; gz::msgs::Boolean rep; bool result;
    req.set_name(name);
    req.mutable_position()->set_x(x); req.mutable_position()->set_y(y); req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_x(qx); req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz); req.mutable_orientation()->set_w(qw);
    g_gz.Request("/world/empty/set_pose",req,200,rep,result);
}

// =============================================================================
// MOTION
// =============================================================================

static void motion_cartesian(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    double dx,double dy,double dz,const std::string& rn)
{
    mg->setMaxVelocityScalingFactor(g_vel_pct/100.0);
    auto p=mg->getCurrentPose().pose;
    p.position.x+=dx; p.position.y+=dy; p.position.z+=dz;
    moveit_msgs::msg::RobotTrajectory traj;
    double f=mg->computeCartesianPath({p},0.005,traj);
    if(f>0.8){mg->execute(traj);set_status(rn+": OK ("+std::to_string((int)(f*100))+"%)"); }
    else     {set_status(rn+": Pfad nicht moeglich ("+std::to_string((int)(f*100))+"%)"); }
}

static void motion_rotation(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    double dr,double dp,double dy,const std::string& rn)
{
    mg->setMaxVelocityScalingFactor(g_vel_pct/100.0);
    auto cur=mg->getCurrentPose().pose; auto t=cur;
    double qx=cur.orientation.x,qy=cur.orientation.y,qz=cur.orientation.z,qw=cur.orientation.w;
    double cr=cos(dr/2),sr=sin(dr/2),cp=cos(dp/2),sp=sin(dp/2),cy=cos(dy/2),sy=sin(dy/2);
    double dw=cr*cp*cy+sr*sp*sy,dxi=sr*cp*cy-cr*sp*sy,dyi=cr*sp*cy+sr*cp*sy,dzi=cr*cp*sy-sr*sp*cy;
    t.orientation.w=qw*dw-qx*dxi-qy*dyi-qz*dzi; t.orientation.x=qw*dxi+qx*dw+qy*dzi-qz*dyi;
    t.orientation.y=qw*dyi-qx*dzi+qy*dw+qz*dxi; t.orientation.z=qw*dzi+qx*dyi-qy*dxi+qz*dw;
    moveit_msgs::msg::RobotTrajectory traj;
    double f=mg->computeCartesianPath({t},0.005,traj);
    if(f>0.8){mg->execute(traj);set_status(rn+": Rotation OK");}
    else     {set_status(rn+": Rotation nicht moeglich");}
}

static bool move_joints(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    std::vector<double> jv,const std::string& label,
    double plan_time=5.0, double vel_scale=-1.0, double acc_scale=1.0)
{
    double vs = (vel_scale>0) ? vel_scale : (g_vel_pct/100.0);
    mg->setMaxVelocityScalingFactor(vs);
    mg->setMaxAccelerationScalingFactor(acc_scale);
    mg->setPlanningTime(plan_time);
    mg->setJointValueTarget(jv);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (mg->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
    mg->setPlanningTime(5.0); // restore default
    if(ok){mg->execute(plan);return true;}
    set_status(label+": Planung fehlgeschlagen"); return false;
}

// =============================================================================
// E-STOP / STOP
// =============================================================================

static void trigger_estop() {
    g_estop=true; g_seq_stop_req=true; g_seq_paused=false;
    g_picking=false; g_picked.clear();
    if(g_arm_mg)   try{g_arm_mg->stop();}   catch(...){}
    if(g_scara_mg) try{g_scara_mg->stop();} catch(...){}
    set_status("!!! E-STOP AKTIVIERT !!!");
}

static void trigger_stop() {
    g_seq_stop_req=true; g_seq_paused=false;
    if(g_arm_mg)   try{g_arm_mg->stop();}   catch(...){}
    if(g_scara_mg) try{g_scara_mg->stop();} catch(...){}
    set_status("Sequenz: Stop angefordert");
}

// =============================================================================
// HOME
// =============================================================================

static void go_home(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
                    const std::string& rn)
{
    mg->setMaxVelocityScalingFactor(g_vel_pct/100.0);
    mg->setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan p;
    if(mg->plan(p)==moveit::core::MoveItErrorCode::SUCCESS){mg->execute(p);set_status(rn+": Home");}
    else set_status(rn+": Home fehlgeschlagen");
    mg->clearPoseTargets();
}

// =============================================================================
// PICK / PLACE / RESET
//   prefer_substr : prefer objects whose name contains this string (e.g. "schraube")
//   max_count     : maximum number of objects to grab (1 = single pick for screw)
// =============================================================================

static void do_pick_impl(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
    double bx,double by,double bz,
    const std::string& prefer_substr="",
    int max_count=999)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto tcp=mg->getCurrentPose().pose;
    double tx=tcp.position.x+bx, ty=tcp.position.y+by, tz=tcp.position.z+bz;

    // Collect candidates within radius, sort (preferred first, then by distance)
    struct Cand { std::string name; double dist; std::array<double,3> pos; };
    std::vector<Cand> cands;
    {
        std::lock_guard<std::mutex> lk(g_obj_mx);
        for(auto&[name,pos]:g_obj_pos){
            double d=std::hypot(std::hypot(pos[0]-tx,pos[1]-ty),pos[2]-tz);
            if(d<=0.50) cands.push_back({name,d,pos});
        }
    }
    if(cands.empty()){set_status("Kein Objekt im Greifradius (300 mm)");return;}

    std::sort(cands.begin(),cands.end(),[&](const Cand&a,const Cand&b){
        bool ap=!prefer_substr.empty()&&a.name.find(prefer_substr)!=std::string::npos;
        bool bp=!prefer_substr.empty()&&b.name.find(prefer_substr)!=std::string::npos;
        if(ap!=bp) return (int)ap>(int)bp;  // preferred first
        return a.dist<b.dist;               // then by distance
    });
    if((int)cands.size()>max_count) cands.resize(max_count);

    // Compute TCP-relative offsets for each candidate
    double tqx=tcp.orientation.x,tqy=tcp.orientation.y,tqz=tcp.orientation.z,tqw=tcp.orientation.w;
    double iqx=-tqx,iqy=-tqy,iqz=-tqz,iqw=tqw;
    std::vector<PickedObj> picked;
    {
        std::lock_guard<std::mutex> lk(g_obj_mx);
        for(auto&c:cands){
            PickedObj po; po.name=c.name;
            double wox=c.pos[0]-tx,woy=c.pos[1]-ty,woz=c.pos[2]-tz;
            double cx=iqy*woz-iqz*woy,cy=iqz*wox-iqx*woz,cz=iqx*woy-iqy*wox;
            double cx2=iqy*cz-iqz*cy,cy2=iqz*cx-iqx*cz,cz2=iqx*cy-iqy*cx;
            po.off_x=wox+2*iqw*cx+2*cx2; po.off_y=woy+2*iqw*cy+2*cy2; po.off_z=woz+2*iqw*cz+2*cz2;
            auto&ori=g_obj_ori[c.name];
            double oqx=ori[0],oqy=ori[1],oqz=ori[2],oqw=ori[3];
            po.off_rqw=iqw*oqw-iqx*oqx-iqy*oqy-iqz*oqz; po.off_rqx=iqw*oqx+iqx*oqw+iqy*oqz-iqz*oqy;
            po.off_rqy=iqw*oqy-iqx*oqz+iqy*oqw+iqz*oqx; po.off_rqz=iqw*oqz+iqx*oqy-iqy*oqx+iqz*oqw;
            picked.push_back(po);
            RCLCPP_INFO(rclcpp::get_logger("rod_hmi"),"Pick: %s  d=%.0f mm",c.name.c_str(),c.dist*1000);
        }
    }

    g_picked=picked; g_picking=true;
    std::string names; for(auto&po:picked) names+=(names.empty()?"":",")+po.name;
    set_status("Greife: "+names);

    std::thread([mg,bx,by,bz](){
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        while(g_picking&&mg){
            auto tcp=mg->getCurrentPose().pose;
            double tqx=tcp.orientation.x,tqy=tcp.orientation.y,tqz=tcp.orientation.z,tqw=tcp.orientation.w;
            for(auto&po:g_picked){
                double cx=tqy*po.off_z-tqz*po.off_y,cy=tqz*po.off_x-tqx*po.off_z,cz=tqx*po.off_y-tqy*po.off_x;
                double cx2=tqy*cz-tqz*cy,cy2=tqz*cx-tqx*cz,cz2=tqx*cy-tqy*cx;
                double nx=tcp.position.x+bx+po.off_x+2*tqw*cx+2*cx2;
                double ny=tcp.position.y+by+po.off_y+2*tqw*cy+2*cy2;
                double nz=tcp.position.z+bz+po.off_z+2*tqw*cz+2*cz2;
                double rw=tqw*po.off_rqw-tqx*po.off_rqx-tqy*po.off_rqy-tqz*po.off_rqz;
                double rx=tqw*po.off_rqx+tqx*po.off_rqw+tqy*po.off_rqz-tqz*po.off_rqy;
                double ry=tqw*po.off_rqy-tqx*po.off_rqz+tqy*po.off_rqw+tqz*po.off_rqx;
                double rz=tqw*po.off_rqz+tqx*po.off_rqy-tqy*po.off_rqx+tqz*po.off_rqw;
                gz_set_pose(po.name,nx,ny,nz,rx,ry,rz,rw);
                {std::lock_guard<std::mutex> lk(g_obj_mx);
                 g_obj_pos[po.name]={nx,ny,nz}; g_obj_ori[po.name]={rx,ry,rz,rw};}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
        }
    }).detach();
}

// Arm pick: all objects within radius
static void do_pick(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
{ do_pick_impl(mg,ARM_OX,ARM_OY,ARM_OZ); }

static void do_place() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // Freeze every picked object at its current tracked world position BEFORE stopping tracking
    {
        std::lock_guard<std::mutex> lk(g_obj_mx);
        for(auto&po:g_picked){
            auto& pos=g_obj_pos[po.name];
            auto& ori=g_obj_ori[po.name];
            gz_set_pose(po.name,pos[0],pos[1],pos[2],ori[0],ori[1],ori[2],ori[3]);
        }
    }
    g_picking=false;
    std::string names; for(auto&po:g_picked) names+=(names.empty()?"":",")+po.name;
    set_status("Abgesetzt: "+names); g_picked.clear();
}
static void reset_scene() {
    g_picking=false; g_picked.clear();
    std::lock_guard<std::mutex> lk(g_obj_mx);
    for(auto&[name,pos]:k_init_pos){
        const auto&ori=k_init_ori.at(name);
        gz_set_pose(name,pos[0],pos[1],pos[2],ori[0],ori[1],ori[2],ori[3]);
        g_obj_pos[name]=pos; g_obj_ori[name]=ori;
    }
    set_status("Szene zurueckgesetzt");
}

// =============================================================================
// SCARA SCREW SEQUENCE
//   Picks exactly ONE object, preferring "schraube_*" by name
// =============================================================================

static void do_screw_sequence(
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg)
{
    set_status("SCARA: Suche Schraube...");

    auto jv_start=mg->getCurrentJointValues();
    if(jv_start.size()<4){set_status("SCARA: Zu wenige Joints");return;}
    double j4_start=jv_start[3];

    // Pick exactly 1 object, prefer those named "schraube_*"
    do_pick_impl(mg,SCARA_OX,SCARA_OY,SCARA_OZ,"schraube",1);
    if(!g_picking){set_status("SCARA: Pick fehlgeschlagen");return;}

    set_status("SCARA: 28 mm runter");
    motion_cartesian(mg,0,0,-0.028,"scara");
    if(g_estop) return;

    set_status("SCARA: J4 dreht (3 Umdrehungen)");
    { auto jv=mg->getCurrentJointValues(); jv[3]+=6.0*M_PI; move_joints(mg,jv,"SCARA J4",0.5,1.0,1.0); }
    if(g_estop) return;

    do_place();
    //std::this_thread::sleep_for(std::chrono::milliseconds(200));

    set_status("SCARA: 28 mm rauf");
    motion_cartesian(mg,0,0,0.028,"scara");
    if(g_estop) return;

    set_status("SCARA: J4 Rueckfahrt");
    { auto jv=mg->getCurrentJointValues(); jv[3]=j4_start; move_joints(mg,jv,"SCARA J4 back",0.5,1.0,1.0); }
    set_status("SCARA: Schrauben abgeschlossen");
}

// =============================================================================
// POSE MANAGEMENT
// =============================================================================

static void save_pose(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg,
                      const std::string& robot,const std::string& name,Action action)
{
    SavedPose sp; auto p=mg->getCurrentPose().pose;
    sp.name=name; sp.robot=robot; sp.action=action;
    sp.x=p.position.x; sp.y=p.position.y; sp.z=p.position.z;
    sp.qx=p.orientation.x; sp.qy=p.orientation.y; sp.qz=p.orientation.z; sp.qw=p.orientation.w;
    sp.joints=mg->getCurrentJointValues();
    {std::lock_guard<std::mutex> lk(g_poses_mx); g_poses.push_back(sp);}
    set_status("Gespeichert: "+name+"  ["+action_str(action)+"]");
}

static void export_poses(const std::string& path) {
    std::ofstream f(path);
    f<<"# ROD Pose Export\n# robot,name,x,y,z,qx,qy,qz,qw,action,j0,...\n";
    std::lock_guard<std::mutex> lk(g_poses_mx);
    for(auto&sp:g_poses){
        f<<sp.robot<<","<<sp.name<<","<<sp.x<<","<<sp.y<<","<<sp.z<<","
         <<sp.qx<<","<<sp.qy<<","<<sp.qz<<","<<sp.qw<<","<<action_str(sp.action);
        for(double j:sp.joints) f<<","<<j;
        f<<"\n";
    }
    set_status("Exportiert: "+path);
}

static void import_poses(const std::string& path) {
    std::ifstream f(path);
    if(!f.is_open()){set_status("Import fehlgeschlagen: "+path);return;}
    std::vector<SavedPose> imported;
    std::string line;
    while(std::getline(f,line)){
        if(line.empty()||line[0]=='#') continue;
        std::vector<std::string> cols; std::stringstream ss(line); std::string tok;
        while(std::getline(ss,tok,',')){
            tok.erase(0,tok.find_first_not_of(" \t\r"));
            if(!tok.empty()) tok.erase(tok.find_last_not_of(" \t\r")+1);
            cols.push_back(tok);
        }
        if(cols.size()<9) continue;
        SavedPose sp;
        try{
            sp.robot=cols[0];sp.name=cols[1];
            sp.x=std::stod(cols[2]);sp.y=std::stod(cols[3]);sp.z=std::stod(cols[4]);
            sp.qx=std::stod(cols[5]);sp.qy=std::stod(cols[6]);
            sp.qz=std::stod(cols[7]);sp.qw=std::stod(cols[8]);
            if(cols.size()>=10) sp.action=action_from_str(cols[9]);
            for(size_t c=10;c<cols.size();c++) try{sp.joints.push_back(std::stod(cols[c]));}catch(...){break;}
        }catch(...){continue;}
        imported.push_back(sp);
    }
    {std::lock_guard<std::mutex> lk(g_poses_mx);
     g_poses.insert(g_poses.end(),imported.begin(),imported.end());}
    set_status("Importiert: "+std::to_string(imported.size())+" Posen aus "+path);
}

// =============================================================================
// SEQUENCE (with Pause / Stop / Loop)
// =============================================================================

static void run_sequence() {
    g_seq_running=true; g_seq_paused=false; g_seq_stop_req=false; g_seq_count=0;

    do {
        g_seq_count++;
        std::vector<SavedPose> seq;
        {std::lock_guard<std::mutex> lk(g_poses_mx); seq=g_poses;}
        set_status("Sequenz (x"+std::to_string(g_seq_count.load())+"): "+std::to_string(seq.size())+" Posen");

        for(size_t i=0;i<seq.size();i++){
            // Check stop/E-Stop
            if(g_seq_stop_req||g_estop){
                g_seq_step=-1; g_seq_running=false;
                set_status("Sequenz gestoppt"); return;
            }
            // Pause: spin-wait
            while(g_seq_paused&&!g_seq_stop_req&&!g_estop)
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
            if(g_seq_stop_req||g_estop){
                g_seq_step=-1; g_seq_running=false;
                set_status("Sequenz gestoppt"); return;
            }

            g_seq_step=(int)i;
            auto&sp=seq[i]; auto mg=(sp.robot=="arm")?g_arm_mg:g_scara_mg;
            if(!mg) continue;
            set_status("Sequenz "+std::to_string(i+1)+"/"+std::to_string(seq.size())+": "+sp.name);

            bool ok=false;
            if(!sp.joints.empty()) ok=move_joints(mg,sp.joints,sp.name);
            if(!ok){
                geometry_msgs::msg::Pose t;
                t.position.x=sp.x;t.position.y=sp.y;t.position.z=sp.z;
                t.orientation.x=sp.qx;t.orientation.y=sp.qy;t.orientation.z=sp.qz;t.orientation.w=sp.qw;
                moveit_msgs::msg::RobotTrajectory traj;
                double frac=mg->computeCartesianPath({t},0.005,traj);
                if(frac>0.5){mg->execute(traj);ok=true;}
                else{mg->setPoseTarget(t);
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    ok=(mg->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
                    if(ok){ mg->execute(plan); }
                    mg->clearPoseTargets();}
            }
            if(!ok){RCLCPP_ERROR(rclcpp::get_logger("rod_hmi"),"[%zu] %s FAILED",i+1,sp.name.c_str());continue;}

            if(g_seq_stop_req||g_estop) break;
            switch(sp.action){
                case Action::Pick:       if(mg) do_pick(mg); break;
                case Action::Place:      do_place(); break;
                case Action::Screw:      if(mg) do_screw_sequence(mg); break;
                case Action::SceneReset: reset_scene(); break;
                case Action::Unscrew:    set_status(sp.robot+": Unscrew (Platzhalter)");
                                         std::this_thread::sleep_for(std::chrono::milliseconds(500)); break;
                default: break;
            }
        }
    } while(g_seq_loop.load()&&!g_seq_stop_req&&!g_estop);

    g_seq_step=-1; g_seq_running=false;
    int cnt=g_seq_count.load();
    std::string sfx=(cnt>1)?" (x"+std::to_string(cnt)+" Loops)":"";
    set_status("Sequenz abgeschlossen"+sfx);
}

// =============================================================================
// ROS THREAD
// =============================================================================

static void ros_thread(int argc,char**argv) {
    rclcpp::init(argc,argv);
    set_status("Starte Simulation...");
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch rod_cell cell.launch.py > /tmp/cell.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(15));
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch arm_moveit move_group.launch.py > /tmp/arm_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::system("bash -c 'source ~/rod_ws/install/setup.bash && ros2 launch scara_moveit move_group.launch.py > /tmp/scara_mg.log 2>&1' &");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    set_status("Verbinde Arm...");
    auto arm_pkg  =ament_index_cpp::get_package_share_directory("arm_moveit");
    auto robot_pkg=ament_index_cpp::get_package_share_directory("robot_arm_6dof_assembly");
    std::string urdf=replace_all(read_file(robot_pkg+"/urdf/robot_arm_6dof_assembly.urdf"),
                                 "$(find robot_arm_6dof_assembly)",robot_pkg);
    std::string srdf=read_file(arm_pkg+"/config/knickarm_6dof.srdf");
    rclcpp::NodeOptions oa; oa.automatically_declare_parameters_from_overrides(true);
    oa.arguments({"--ros-args","-r","joint_states:=/arm/joint_states"});
    oa.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
                             rclcpp::Parameter("robot_description",urdf),
                             rclcpp::Parameter("robot_description_semantic",srdf)});
    auto arm_node=rclcpp::Node::make_shared("hmi_arm_node","/arm_hmi",oa);
    g_arm_mg=std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        arm_node,moveit::planning_interface::MoveGroupInterface::Options("arm","robot_description","/arm"));
    g_arm_mg->setMaxVelocityScalingFactor(g_vel_pct/100.0);
    g_arm_mg->setMaxAccelerationScalingFactor(1.0);
    g_arm_ready=true;

    set_status("Verbinde SCARA...");
    auto scara_pkg =ament_index_cpp::get_package_share_directory("scara_moveit");
    auto scara4_pkg=ament_index_cpp::get_package_share_directory("scara_4");
    std::string surdf=replace_all(read_file(scara4_pkg+"/urdf/SCARA_4.urdf"),"$(find scara_4)",scara4_pkg);
    std::string ssrdf=read_file(scara_pkg+"/config/scara.srdf");
    rclcpp::NodeOptions os; os.automatically_declare_parameters_from_overrides(true);
    os.arguments({"--ros-args","-r","joint_states:=/scara/joint_states"});
    os.parameter_overrides({rclcpp::Parameter("use_sim_time",true),
                             rclcpp::Parameter("scara_description",surdf),
                             rclcpp::Parameter("scara_description_semantic",ssrdf)});
    auto scara_node=rclcpp::Node::make_shared("hmi_scara_node","/scara_hmi",os);
    g_scara_mg=std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        scara_node,moveit::planning_interface::MoveGroupInterface::Options("scara","scara_description","/scara"));
    g_scara_mg->setMaxVelocityScalingFactor(g_vel_pct/100.0);
    g_scara_mg->setMaxAccelerationScalingFactor(1.0);
    g_scara_ready=true;

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(arm_node); exec.add_node(scara_node);
    g_ros_running=true; set_status("Bereit");

    std::thread([](){
        while(g_ros_running){
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if(g_arm_mg){auto jv=g_arm_mg->getCurrentJointValues();
                std::lock_guard<std::mutex> lk(g_jv_mx);
                for(int i=0;i<6&&i<(int)jv.size();i++) g_jv_arm[i]=(float)jv[i];}
            if(g_scara_mg){auto jv=g_scara_mg->getCurrentJointValues();
                std::lock_guard<std::mutex> lk(g_jv_mx);
                for(int i=0;i<4&&i<(int)jv.size();i++) g_jv_scara[i]=(float)jv[i];}
        }
    }).detach();
    exec.spin(); rclcpp::shutdown();
}

// Kills all background processes started by the HMI.
// Called both from the Quit button and from the end of main().
// Two-phase: SIGTERM first, then SIGKILL for anything still alive.
static void full_shutdown()
{
    // Phase 1 – polite SIGTERM
    std::system("pkill -f 'ros2 launch rod_cell'   2>/dev/null");
    std::system("pkill -f 'ros2 launch arm_moveit'  2>/dev/null");
    std::system("pkill -f 'ros2 launch scara_moveit' 2>/dev/null");
    std::system("pkill -f 'move_group'              2>/dev/null");
    std::system("pkill -f 'robot_state_publisher'   2>/dev/null");
    std::system("pkill -f 'ros2_control_node'       2>/dev/null");
    std::system("pkill -f 'spawn_entity'            2>/dev/null");
    std::system("pkill -f 'gz sim'                  2>/dev/null");
    std::system("pkill -f 'gz_ros2_control'         2>/dev/null");
    // Give processes a moment to exit cleanly
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    // Phase 2 – SIGKILL for anything still running
    std::system("pkill -9 -f 'gz sim'              2>/dev/null");
    std::system("pkill -9 -f 'move_group'          2>/dev/null");
    std::system("pkill -9 -f 'gz_ros2_control'     2>/dev/null");
    // Reset ROS2 daemon so the next start finds a clean state
    std::system("ros2 daemon stop 2>/dev/null");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::system("ros2 daemon start 2>/dev/null");
}


// =============================================================================

static void apply_theme() {
    ImGuiStyle&s=ImGui::GetStyle();
    s.WindowRounding=0;s.ChildRounding=3;s.FrameRounding=2;s.GrabRounding=2;s.TabRounding=2;
    s.WindowBorderSize=1;s.FrameBorderSize=0;
    s.ItemSpacing={8,5};s.ItemInnerSpacing={6,4};s.FramePadding={6,4};
    s.WindowPadding={10,8};s.IndentSpacing=12;s.ScrollbarSize=9;s.GrabMinSize=8;
    auto&c=s.Colors;
    c[ImGuiCol_WindowBg]         ={0.090f,0.098f,0.122f,1};
    c[ImGuiCol_ChildBg]          ={0.118f,0.126f,0.157f,1};
    c[ImGuiCol_PopupBg]          ={0.118f,0.126f,0.157f,1};
    c[ImGuiCol_FrameBg]          ={0.071f,0.075f,0.102f,1};
    c[ImGuiCol_FrameBgHovered]   ={0.141f,0.153f,0.196f,1};
    c[ImGuiCol_FrameBgActive]    ={0.180f,0.196f,0.243f,1};
    c[ImGuiCol_Text]             ={0.929f,0.941f,0.961f,1};
    c[ImGuiCol_TextDisabled]     ={0.420f,0.447f,0.510f,1};
    c[ImGuiCol_Border]           ={0.180f,0.200f,0.251f,1};
    c[ImGuiCol_BorderShadow]     ={0,0,0,0};
    c[ImGuiCol_TitleBg]          ={0.055f,0.059f,0.075f,1};
    c[ImGuiCol_TitleBgActive]    ={0.055f,0.059f,0.075f,1};
    c[ImGuiCol_TitleBgCollapsed] ={0.055f,0.059f,0.075f,.5f};
    c[ImGuiCol_Header]           ={0.141f,0.161f,0.212f,1};
    c[ImGuiCol_HeaderHovered]    ={0.188f,0.216f,0.275f,1};
    c[ImGuiCol_HeaderActive]     ={0.235f,0.267f,0.333f,1};
    c[ImGuiCol_Button]           ={0.165f,0.184f,0.235f,1};
    c[ImGuiCol_ButtonHovered]    ={0.910f,0.627f,0.125f,1};
    c[ImGuiCol_ButtonActive]     ={1.000f,0.733f,0.251f,1};
    c[ImGuiCol_SliderGrab]       ={0.910f,0.627f,0.125f,1};
    c[ImGuiCol_SliderGrabActive] ={1.000f,0.733f,0.251f,1};
    c[ImGuiCol_CheckMark]        ={0.910f,0.627f,0.125f,1};
    c[ImGuiCol_ResizeGrip]       ={0.910f,0.627f,0.125f,.2f};
    c[ImGuiCol_ResizeGripHovered]={0.910f,0.627f,0.125f,.65f};
    c[ImGuiCol_ResizeGripActive] ={0.910f,0.627f,0.125f,.95f};
    c[ImGuiCol_ScrollbarBg]      ={0.055f,0.059f,0.075f,1};
    c[ImGuiCol_ScrollbarGrab]    ={0.180f,0.200f,0.251f,1};
    c[ImGuiCol_ScrollbarGrabHovered]={0.259f,0.286f,0.349f,1};
    c[ImGuiCol_ScrollbarGrabActive] ={0.910f,0.627f,0.125f,1};
    c[ImGuiCol_Separator]        ={0.180f,0.200f,0.251f,1};
    c[ImGuiCol_SeparatorHovered] ={0.910f,0.627f,0.125f,.8f};
    c[ImGuiCol_SeparatorActive]  ={0.910f,0.627f,0.125f,1};
    c[ImGuiCol_TableHeaderBg]    ={0.055f,0.059f,0.075f,1};
    c[ImGuiCol_TableBorderStrong]={0.180f,0.200f,0.251f,1};
    c[ImGuiCol_TableBorderLight] ={0.141f,0.161f,0.212f,1};
    c[ImGuiCol_TableRowBg]       ={0,0,0,0};
    c[ImGuiCol_TableRowBgAlt]    ={1,1,1,.02f};
    c[ImGuiCol_NavHighlight]     ={0.910f,0.627f,0.125f,1};
}

// =============================================================================
// GUI HELPERS
// =============================================================================

static constexpr ImVec4 COL_AMBER={0.910f,0.627f,0.125f,1};
static constexpr ImVec4 COL_GREEN={0.267f,0.784f,0.478f,1};
static constexpr ImVec4 COL_RED  ={0.784f,0.310f,0.310f,1};
static constexpr ImVec4 COL_DIM  ={0.490f,0.525f,0.588f,1};
static constexpr ImVec4 COL_WARN ={0.941f,0.706f,0.200f,1};

static void sec(const char*label) {
    ImDrawList*dl=ImGui::GetWindowDrawList();
    ImVec2 p=ImGui::GetCursorScreenPos(); float lh=ImGui::GetTextLineHeight();
    dl->AddRectFilled(p,{p.x+3.0f,p.y+lh+2.0f},IM_COL32(232,160,32,255));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX()+8.0f);
    ImGui::TextColored({0.710f,0.733f,0.773f,1.0f},"%s",label);
    ImGui::Spacing();
}

static void led(const char*label,bool on) {
    ImDrawList*dl=ImGui::GetWindowDrawList();
    ImVec2 p=ImGui::GetCursorScreenPos(); float lh=ImGui::GetTextLineHeight(),r=5.0f;
    ImVec2 ctr={p.x+r,p.y+lh*0.5f+1.0f};
    dl->AddCircleFilled(ctr,r,on?IM_COL32(68,200,122,255):IM_COL32(180,64,64,255),12);
    dl->AddCircle(ctr,r,IM_COL32(0,0,0,70),12,1.0f);
    ImGui::Dummy({r*2+1,lh}); ImGui::SameLine(0,4);
    ImGui::TextColored(on?COL_GREEN:COL_RED,"%s",label);
}

static bool jbtn(const char*lbl,float w=54.0f,float h=36.0f)
{ return ImGui::Button(lbl,{w,h}); }

// Joint slider — returns {is_active, deactivated_after_edit} captured immediately
// after SliderFloat, before the degree label (TextColored would pollute IsItemActive).
struct SliderState { bool active; bool done; };

static SliderState joint_slider(const char*id,float*val,float lo,float hi,float sw,bool linear=false)
{
    if(linear){
        float v_mm=*val*1000.0f;
        ImGui::SetNextItemWidth(sw);
        if(ImGui::SliderFloat(id,&v_mm,lo*1000.0f,hi*1000.0f,"%.1f mm"))
            *val=v_mm/1000.0f;
    } else {
        ImGui::SetNextItemWidth(sw);
        ImGui::SliderFloat(id,val,lo,hi,"%.3f");
    }
    // Capture states NOW — before any other ImGui item is rendered
    SliderState st{ImGui::IsItemActive(), ImGui::IsItemDeactivatedAfterEdit()};
    // Degree/mm label (purely visual, does not affect item state)
    if(!linear){
        ImGui::SameLine(0,6);
        ImGui::TextColored(COL_DIM,"%+7.1f\xc2\xb0",*val*180.0f/(float)M_PI);
    }
    return st;
}

// =============================================================================
// MAIN
// =============================================================================

int main(int argc,char**argv) {
    std::thread(ros_thread,argc,argv).detach();

    if(!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow*win=glfwCreateWindow(1024,1012,"ROD Cell Control",nullptr,nullptr);
    if(!win){glfwTerminate();return -1;}
    glfwMakeContextCurrent(win); glfwSwapInterval(1);

    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGui::StyleColorsDark(); apply_theme();
    ImGui_ImplGlfw_InitForOpenGL(win,true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::GetIO().FontGlobalScale=1.1f;

    int   robot=0;
    float step_xy=0.05f,step_z=0.05f,rot_step=0.10f;
    char  pose_name[64]="Pose_1";
    int   sel_action=0;
    static char csv_path[256]={}; static bool csv_init=false;

    float arm_edit[6]={}; bool arm_drag[6]={};
    float scara_edit[4]={}; bool scara_drag[4]={};
    int   prev_sel=-1;
    static size_t prev_log_sz=0;

    double tcp_x=0,tcp_y=0,tcp_z=0; std::mutex tcp_mx;
    std::thread([&](){
        while(!glfwWindowShouldClose(win)){
            if(g_ros_running){
                auto mg=(robot==0)?g_arm_mg:g_scara_mg;
                if(mg) try{
                    auto p=mg->getCurrentPose().pose;
                    std::lock_guard<std::mutex> lk(tcp_mx);
                    tcp_x=p.position.x;tcp_y=p.position.y;tcp_z=p.position.z;
                }catch(...){}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }).detach();

    auto mg    =[&](){return(robot==0)?g_arm_mg:g_scara_mg;};
    auto rname =[&]()->std::string{return(robot==0)?"arm":"scara";};
    const ImGuiWindowFlags NO_SCROLL=
        ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse;

    // Helper: map combo index → Action
    auto idx2act=[&](int i)->Action{
        if(robot==0){if(i==1)return Action::Pick;if(i==2)return Action::Place;if(i==3)return Action::SceneReset;}
        else        {if(i==1)return Action::Screw;if(i==2)return Action::Unscrew;if(i==3)return Action::SceneReset;}
        return Action::None;
    };
    auto act2idx=[&](Action a)->int{
        if(robot==0){if(a==Action::Pick)return 1;if(a==Action::Place)return 2;if(a==Action::SceneReset)return 3;}
        else        {if(a==Action::Screw)return 1;if(a==Action::Unscrew)return 2;if(a==Action::SceneReset)return 3;}
        return 0;
    };

    // =========================================================================
    while(!glfwWindowShouldClose(win)){
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        const ImVec2 disp=ImGui::GetIO().DisplaySize;
        ImGui::SetNextWindowPos({0,0},ImGuiCond_Always);
        ImGui::SetNextWindowSize(disp,ImGuiCond_Always);
        ImGui::Begin("##root",nullptr,
            ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
            ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoBringToFrontOnFocus);

        const bool rdy=g_ros_running.load()&&!g_estop.load();
        const bool running=g_seq_running.load();
        const bool paused =g_seq_paused.load();
        const bool estop  =g_estop.load();

        // Sync pose editor fields on selection change
        if(g_sel!=prev_sel){
            std::lock_guard<std::mutex> lk(g_poses_mx);
            if(g_sel<(int)g_poses.size()){
                auto&sp=g_poses[g_sel];
                snprintf(pose_name,sizeof(pose_name),"%s",sp.name.c_str());
                sel_action=act2idx(sp.action);
            }
            prev_sel=g_sel;
        }

        // ── HEADER ROW 1 ────────────────────────────────────────────
        if(estop) ImGui::TextColored({1.0f,0.3f,0.3f,1.0f},"!!! E-STOP AKTIV !!!");
        else      ImGui::TextColored(COL_AMBER,"ROD CELL CONTROL");
        ImGui::SameLine(0,20); led("ARM",g_arm_ready.load());
        ImGui::SameLine(0,14); led("SCARA",g_scara_ready.load());
        if(g_picking){ImGui::SameLine(0,14);ImGui::TextColored(COL_WARN,"[G]  %zu gegriffen",g_picked.size());}
        {
            const float quit_w=50.0f, gap=6.0f;
            float bw=estop?100.0f:80.0f;
            // Quit button — rightmost
            ImGui::SameLine(disp.x-(quit_w+bw+gap)-ImGui::GetStyle().WindowPadding.x*2-4.0f);
            ImGui::PushStyleColor(ImGuiCol_Button,       {0.200f,0.200f,0.200f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.380f,0.380f,0.380f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.520f,0.520f,0.520f,1});
            if(ImGui::Button("Quit",{quit_w,22})){
                set_status("Beende...");
                g_ros_running = false;
                g_seq_stop_req = true;
                std::thread([win]{ full_shutdown(); glfwSetWindowShouldClose(win,GLFW_TRUE); }).detach();
            }
            ImGui::PopStyleColor(3);
            // E-Stop — left of Quit
            ImGui::SameLine(0,gap);
            if(estop){
                ImGui::PushStyleColor(ImGuiCol_Button,       {0.502f,0.090f,0.090f,1});
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.651f,0.129f,0.129f,1});
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.753f,0.176f,0.176f,1});
                if(ImGui::Button("E-Stop Reset",{bw,22})){g_estop=false;set_status("E-Stop zurueckgesetzt");}
                ImGui::PopStyleColor(3);
            } else {
                ImGui::PushStyleColor(ImGuiCol_Button,       {0.502f,0.059f,0.059f,1});
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.651f,0.090f,0.090f,1});
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.800f,0.130f,0.130f,1});
                if(ImGui::Button("E-STOP",{bw,22})) trigger_estop();
                ImGui::PopStyleColor(3);
            }
        }

        // ── HEADER ROW 2: Velocity + TCP ────────────────────────────
        ImGui::TextColored(COL_DIM,"Geschw."); ImGui::SameLine(0,6);
        ImGui::SetNextItemWidth(180);
        if(ImGui::SliderFloat("##vel",&g_vel_pct,5.0f,100.0f,"%.0f %%")){
            float vs=g_vel_pct/100.0f;
            std::thread([vs](){
                if(g_arm_mg)   g_arm_mg->setMaxVelocityScalingFactor(vs);
                if(g_scara_mg) g_scara_mg->setMaxVelocityScalingFactor(vs);
            }).detach();
        }
        {std::lock_guard<std::mutex> lk(tcp_mx);
         char buf[80]; snprintf(buf,sizeof(buf),"TCP  x%+.3f  y%+.3f  z%+.3f",tcp_x,tcp_y,tcp_z);
         float tw=ImGui::CalcTextSize(buf).x;
         ImGui::SameLine(disp.x-tw-ImGui::GetStyle().WindowPadding.x*2);
         ImGui::TextColored(COL_DIM,"%s",buf);}

        // ── STATUS BAR ───────────────────────────────────────────────
        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.055f,0.059f,0.075f,1.0f});
        ImGui::BeginChild("##status",{0,22},false);
        ImGui::SetCursorPosY(3); ImGui::TextColored(COL_AMBER,">> "); ImGui::SameLine();
        {std::lock_guard<std::mutex> lk(g_status_mx);ImGui::TextUnformatted(g_status.c_str());}
        ImGui::EndChild(); ImGui::PopStyleColor();
        ImGui::Spacing();

        // ── ROBOT SELECTOR ───────────────────────────────────────────
        ImGui::TextColored(COL_DIM,"ROBOTER"); ImGui::SameLine(0,10);
        ImGui::RadioButton("Knickarm",&robot,0); ImGui::SameLine();
        ImGui::RadioButton("SCARA",&robot,1);
        ImGui::Separator(); ImGui::Spacing();

        const float avail=ImGui::GetContentRegionAvail().x;
        const float col_lw=avail*0.415f, col_rw=avail-col_lw-8.0f;
        const float col_h=504.0f;

        // ═══ LEFT COLUMN ════════════════════════════════════════════
        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.118f,0.126f,0.157f,1.0f});
        ImGui::BeginChild("##left",{col_lw,col_h},true,NO_SCROLL);

        sec("TCP-TRANSLATION");
        ImGui::SetNextItemWidth(col_lw-24);
        ImGui::SliderFloat("##sxy",&step_xy,0.001f,0.20f,"XY %.3f m");
        ImGui::SetNextItemWidth(col_lw-24);
        ImGui::SliderFloat("##sz", &step_z, 0.001f,0.20f,"Z  %.3f m");
        ImGui::Spacing();

        ImGui::BeginDisabled(!rdy);
        {const float B=52.0f,G=4.0f,OX=26.0f,OY=ImGui::GetCursorPosY();
         ImGui::SetCursorPos({OX+B+G,OY});
         if(jbtn("Z+"))std::thread([mg,step_z,rname]{motion_cartesian(mg(),0,0,step_z,rname());}).detach();
         ImGui::SetCursorPos({OX,OY+B+G});
         if(jbtn("Y+"))std::thread([mg,step_xy,rname]{motion_cartesian(mg(),0,step_xy,0,rname());}).detach();
         ImGui::SetCursorPos({OX+B+G,OY+B+G});
         if(jbtn("X+"))std::thread([mg,step_xy,rname]{motion_cartesian(mg(),step_xy,0,0,rname());}).detach();
         ImGui::SetCursorPos({OX+(B+G)*2,OY+B+G});
         if(jbtn("X-"))std::thread([mg,step_xy,rname]{motion_cartesian(mg(),-step_xy,0,0,rname());}).detach();
         ImGui::SetCursorPos({OX+(B+G)*3,OY+B+G});
         if(jbtn("Y-"))std::thread([mg,step_xy,rname]{motion_cartesian(mg(),0,-step_xy,0,rname());}).detach();
         ImGui::SetCursorPos({OX+B+G,OY+(B+G)*2});
         if(jbtn("Z-"))std::thread([mg,step_z,rname]{motion_cartesian(mg(),0,0,-step_z,rname());}).detach();
         ImGui::SetCursorPosY(OY+(B+G)*3+6);}
        ImGui::EndDisabled();

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        if(robot==0){
            sec("TCP-ROTATION  (Arm)");
            ImGui::SetNextItemWidth(col_lw-100);
            ImGui::SliderFloat("##rs",&rot_step,0.02f,0.50f,"%.3f rad");
            ImGui::SameLine(0,6);
            ImGui::TextColored(COL_DIM,"%+6.1f\xc2\xb0",rot_step*180.0f/(float)M_PI);
            ImGui::Spacing();
            ImGui::BeginDisabled(!rdy);
            {const float BW=50.0f,BH=30.0f;
             if(jbtn("R+",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),rot_step,0,0,rname());}).detach();
             ImGui::SameLine();
             if(jbtn("R-",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),-rot_step,0,0,rname());}).detach();
             ImGui::SameLine(0,10);
             if(jbtn("P+",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),0,rot_step,0,rname());}).detach();
             ImGui::SameLine();
             if(jbtn("P-",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),0,-rot_step,0,rname());}).detach();
             ImGui::SameLine(0,10);
             if(jbtn("Y+##r",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),0,0,rot_step,rname());}).detach();
             ImGui::SameLine();
             if(jbtn("Y-##r",BW,BH))std::thread([mg,rot_step,rname]{motion_rotation(mg(),0,0,-rot_step,rname());}).detach();
             ImGui::TextColored(COL_DIM,"  Roll         Pitch        Yaw");}
            ImGui::EndDisabled();
            ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        }

        sec("AKTIONEN");
        ImGui::BeginDisabled(!rdy);
        if(robot==0){
            ImGui::PushStyleColor(ImGuiCol_Button,       {0.100f,0.231f,0.141f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.141f,0.318f,0.192f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.180f,0.400f,0.243f,1});
            if(ImGui::Button("PICK",{80,34})){auto m=g_arm_mg;std::thread([m]{do_pick(m);}).detach();}
            ImGui::PopStyleColor(3); ImGui::SameLine();
            ImGui::BeginDisabled(!g_picking);
            ImGui::PushStyleColor(ImGuiCol_Button,       {0.220f,0.102f,0.102f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.302f,0.141f,0.141f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.380f,0.180f,0.180f,1});
            if(ImGui::Button("PLACE",{80,34})) std::thread([]{do_place();}).detach();
            ImGui::PopStyleColor(3); ImGui::EndDisabled();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button,       {0.100f,0.200f,0.231f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.141f,0.275f,0.318f,1});
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.180f,0.349f,0.400f,1});
            if(ImGui::Button("SCREW",{90,34})){auto m=g_scara_mg;std::thread([m]{do_screw_sequence(m);}).detach();}
            ImGui::PopStyleColor(3); ImGui::SameLine();
            if(ImGui::Button("UNSCREW",{90,34})) std::thread([]{set_status("SCARA: Unscrew (Platzhalter)");}).detach();
        }
        ImGui::SameLine(0,8);
        if(ImGui::Button("Home",{60,34})){auto m=mg();std::string rn=rname();std::thread([m,rn]{go_home(m,rn);}).detach();}
        ImGui::SameLine(0,8);
        ImGui::PushStyleColor(ImGuiCol_Button,       {0.240f,0.196f,0.071f,1});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.420f,0.318f,0.086f,1});
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.549f,0.412f,0.110f,1});
        if(ImGui::Button("Scene Reset",{108,34})) std::thread([]{reset_scene();}).detach();
        ImGui::PopStyleColor(3);
        ImGui::EndDisabled();

        ImGui::EndChild(); ImGui::PopStyleColor();
        ImGui::SameLine(0,8);

        // ═══ RIGHT COLUMN ═══════════════════════════════════════════
        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.118f,0.126f,0.157f,1.0f});
        ImGui::BeginChild("##right",{col_rw,col_h},true,NO_SCROLL);

        sec("POSE SPEICHERN / BEARBEITEN");
        ImGui::SetNextItemWidth(128); ImGui::InputText("##pname",pose_name,sizeof(pose_name));
        ImGui::SameLine(0,6); ImGui::TextColored(COL_DIM,"Aktion"); ImGui::SameLine(0,4);
        ImGui::SetNextItemWidth(96);
        if(robot==0){const char*o[]={"--","Pick","Place","Reset"};if(sel_action>3)sel_action=0;ImGui::Combo("##act",&sel_action,o,4);}
        else        {const char*o[]={"--","Screw","Unscrew","Reset"};if(sel_action>3)sel_action=0;ImGui::Combo("##act",&sel_action,o,4);}
        ImGui::SameLine(0,6);
        ImGui::BeginDisabled(!rdy);
        if(ImGui::Button("Speichern",{82,24})){
            std::string pn(pose_name),rn(rname()); auto m=mg(); Action act=idx2act(sel_action);
            std::thread([m,pn,rn,act]{save_pose(m,rn,pn,act);}).detach();
            std::string s=pose_name; size_t di=s.size();
            while(di>0&&std::isdigit(s[di-1]))di--;
            if(di<s.size()) snprintf(pose_name,sizeof(pose_name),"%s%d",s.substr(0,di).c_str(),std::stoi(s.substr(di))+1);
            sel_action=0;
        }
        ImGui::SameLine(0,4);
        if(ImGui::Button("Aendern",{64,24})){
            Action act=idx2act(sel_action);
            std::lock_guard<std::mutex> lk(g_poses_mx);
            if(g_sel<(int)g_poses.size()){
                g_poses[g_sel].name=pose_name; g_poses[g_sel].action=act;
                set_status("Geaendert: "+std::string(pose_name));
            }
        }
        ImGui::EndDisabled();

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        sec("BIBLIOTHEK");

        int pending_a=-1,pending_b=-1;
        {
            ImGui::BeginChild("##ptab",{0,152},false);
            if(ImGui::BeginTable("##pt",7,
                ImGuiTableFlags_RowBg|ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_SizingFixedFit))
            {
                ImGui::TableSetupColumn("#",    ImGuiTableColumnFlags_WidthFixed,20);
                ImGui::TableSetupColumn("Rob",  ImGuiTableColumnFlags_WidthFixed,36);
                ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("xyz",  ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Akt.", ImGuiTableColumnFlags_WidthFixed,46);
                ImGui::TableSetupColumn("J",    ImGuiTableColumnFlags_WidthFixed,16);
                ImGui::TableSetupColumn("",     ImGuiTableColumnFlags_WidthFixed,50);
                ImGui::TableHeadersRow();
                std::lock_guard<std::mutex> lk(g_poses_mx);
                int sz=(int)g_poses.size(), cur_step=g_seq_step.load();
                for(int i=0;i<sz;i++){
                    auto&sp=g_poses[i];
                    ImGui::TableNextRow();
                    if(i==cur_step)
                        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,IM_COL32(232,160,32,60));
                    ImGui::TableSetColumnIndex(0); ImGui::TextColored(COL_DIM,"%d",i+1);
                    ImGui::TableSetColumnIndex(1); ImGui::TextColored(COL_DIM,"%s",sp.robot.c_str());
                    ImGui::TableSetColumnIndex(2);
                    if(ImGui::Selectable(sp.name.c_str(),g_sel==i,
                        ImGuiSelectableFlags_SpanAllColumns|ImGuiSelectableFlags_AllowOverlap))
                        g_sel=i;
                    ImGui::TableSetColumnIndex(3); ImGui::Text("%.3f %.3f %.3f",sp.x,sp.y,sp.z);
                    ImGui::TableSetColumnIndex(4); ImGui::TextColored(COL_DIM,"%s",action_str(sp.action));
                    ImGui::TableSetColumnIndex(5);
                    if(sp.joints.empty())ImGui::TextColored(COL_RED,"-");
                    else                 ImGui::TextColored(COL_GREEN,"ok");
                    ImGui::TableSetColumnIndex(6);
                    ImGui::PushID(i);
                    ImGui::BeginDisabled(i==0);
                    if(ImGui::ArrowButton("##up",ImGuiDir_Up)){pending_a=i-1;pending_b=i;}
                    ImGui::EndDisabled(); ImGui::SameLine(0,2);
                    ImGui::BeginDisabled(i==sz-1);
                    if(ImGui::ArrowButton("##dn",ImGuiDir_Down)){pending_a=i;pending_b=i+1;}
                    ImGui::EndDisabled();
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            ImGui::EndChild();
        }
        if(pending_a>=0){
            std::lock_guard<std::mutex> lk(g_poses_mx);
            if(pending_a<(int)g_poses.size()&&pending_b<(int)g_poses.size()){
                std::swap(g_poses[pending_a],g_poses[pending_b]);
                if(g_sel==pending_a)g_sel=pending_b; else if(g_sel==pending_b)g_sel=pending_a;
            }
        }
        ImGui::BeginDisabled(g_poses.empty());
        if(ImGui::Button("Duplikat",{68,20})){
            std::lock_guard<std::mutex> lk(g_poses_mx);
            if(g_sel<(int)g_poses.size()){
                SavedPose copy=g_poses[g_sel]; copy.name+="_2";
                g_poses.insert(g_poses.begin()+g_sel+1,copy); g_sel++;
                set_status("Dupliziert: "+copy.name);
            }
        }
        ImGui::EndDisabled();

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        sec("KONFIGURATION");
        {
            std::vector<std::string> nv;
            {std::lock_guard<std::mutex> lk(g_poses_mx);
             for(auto&sp:g_poses) nv.push_back("["+sp.robot+"] "+sp.name);}
            if(!nv.empty()){
                g_sel=std::min(g_sel,(int)nv.size()-1);
                std::vector<const char*>cn; for(auto&n:nv)cn.push_back(n.c_str());
                ImGui::SetNextItemWidth(178); ImGui::Combo("##kfg",&g_sel,cn.data(),(int)cn.size());
                ImGui::SameLine(0,6);
                ImGui::BeginDisabled(!rdy);
                if(ImGui::Button("Anfahren",{76,22})){
                    SavedPose sc;{std::lock_guard<std::mutex> lk(g_poses_mx);sc=g_poses[g_sel];}
                    std::thread([sc]{
                        auto m=(sc.robot=="arm")?g_arm_mg:g_scara_mg; if(!m)return;
                        m->setMaxVelocityScalingFactor(g_vel_pct/100.0);
                        if(!sc.joints.empty()) m->setJointValueTarget(sc.joints);
                        else{geometry_msgs::msg::Pose t;
                            t.position.x=sc.x;t.position.y=sc.y;t.position.z=sc.z;
                            t.orientation.x=sc.qx;t.orientation.y=sc.qy;t.orientation.z=sc.qz;t.orientation.w=sc.qw;
                            m->setPoseTarget(t);}
                        moveit::planning_interface::MoveGroupInterface::Plan p;
                        if(m->plan(p)==moveit::core::MoveItErrorCode::SUCCESS){m->execute(p);set_status("Angefahren: "+sc.name);}
                        else set_status("Anfahren fehlgeschlagen: "+sc.name);
                        m->clearPoseTargets();
                    }).detach();
                }
                ImGui::SameLine(0,4);
                if(ImGui::Button("Konfig",{52,22})){
                    std::lock_guard<std::mutex> lk(g_poses_mx); auto&sp=g_poses[g_sel];
                    auto m=(sp.robot=="arm")?g_arm_mg:g_scara_mg;
                    if(m){sp.joints=m->getCurrentJointValues();set_status("Konfig: "+sp.name);}
                }
                bool is_arm=false;
                {std::lock_guard<std::mutex> lk(g_poses_mx);if(!g_poses.empty())is_arm=(g_poses[g_sel].robot=="arm");}
                if(is_arm){
                    ImGui::SameLine(0,4);
                    if(ImGui::Button("Elbow",{48,22})){
                        std::thread([]{
                            if(!g_arm_mg)return;
                            auto jv=g_arm_mg->getCurrentJointValues();
                            if(jv.size()>=3){jv[2]=jv[2]>0?jv[2]-M_PI:jv[2]+M_PI;move_joints(g_arm_mg,jv,"Elbow Flip");}
                        }).detach();
                    }
                }
                ImGui::EndDisabled();
                {std::lock_guard<std::mutex> lk(g_poses_mx);
                 if(!g_poses.empty()){auto&sp=g_poses[g_sel];
                   if(sp.joints.empty())ImGui::TextColored(COL_WARN,"  Keine Konfig");
                   else ImGui::TextColored(COL_GREEN,"  %zu Joints OK",sp.joints.size());}}
            } else ImGui::TextColored(COL_DIM,"Keine Posen vorhanden");
        }

        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
        sec("SEQUENZ");

        // Row 1: playback controls
        ImGui::BeginDisabled(running||g_poses.empty()||!rdy);
        if(ImGui::Button("Starten",{68,24})){
            g_seq_paused=false; g_seq_stop_req=false;
            std::thread(run_sequence).detach();
        }
        ImGui::EndDisabled();
        ImGui::SameLine(0,4);
        ImGui::BeginDisabled(!running);
        const char*pause_lbl=paused?"Resume":"Pause";
        if(ImGui::Button(pause_lbl,{66,24})) g_seq_paused.store(!paused);
        ImGui::SameLine(0,4);
        ImGui::PushStyleColor(ImGuiCol_Button,       {0.380f,0.090f,0.090f,1});
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,{0.502f,0.110f,0.110f,1});
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, {0.651f,0.141f,0.141f,1});
        if(ImGui::Button("Stop",{52,24})) trigger_stop();
        ImGui::PopStyleColor(3);
        ImGui::EndDisabled();
        ImGui::SameLine(0,10);
        {bool lp=g_seq_loop.load(); if(ImGui::Checkbox("Loop",&lp)) g_seq_loop.store(lp);}
        int sc=g_seq_count.load();
        if(sc>0){ImGui::SameLine(0,6);ImGui::TextColored(COL_DIM,"x%d",sc);}

        // Row 2: library management
        ImGui::Spacing();
        ImGui::BeginDisabled(g_poses.empty());
        if(ImGui::Button("Alle loeschen",{108,24})){
            std::lock_guard<std::mutex> lk(g_poses_mx); g_poses.clear();
            snprintf(pose_name,sizeof(pose_name),"Pose_1"); set_status("Alle Posen geloescht");
        }
        ImGui::SameLine(0,4);
        if(ImGui::Button("Sel. loeschen",{102,24})){
            std::lock_guard<std::mutex> lk(g_poses_mx);
            if(g_sel<(int)g_poses.size()){
                std::string n=g_poses[g_sel].name; g_poses.erase(g_poses.begin()+g_sel);
                if(g_sel>=(int)g_poses.size()&&g_sel>0) g_sel--;
                set_status("Geloescht: "+n);
            }
        }
        ImGui::EndDisabled();

        if(!csv_init){
            const char*home=getenv("HOME");
            snprintf(csv_path,sizeof(csv_path),"%s/rod_ws/src/rod_hmi/rod_poses.csv",home?home:"/tmp");
            csv_init=true;
        }
        ImGui::Spacing();
        ImGui::SetNextItemWidth(col_rw-146); ImGui::InputText("##csv",csv_path,sizeof(csv_path));
        ImGui::SameLine(0,4);
        ImGui::BeginDisabled(!rdy);
        if(ImGui::Button("Export",{60,22})){std::string p(csv_path);std::thread([p]{export_poses(p);}).detach();}
        ImGui::SameLine(0,4);
        if(ImGui::Button("Import",{60,22})){std::string p(csv_path);std::thread([p]{import_poses(p);}).detach();}
        ImGui::EndDisabled();

        ImGui::EndChild(); ImGui::PopStyleColor();
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        // ── BOTTOM ROW: JOINTS + OBJECTS ─────────────────────────────
        const float bot_h=(robot==0)?212.0f:156.0f;
        const float jnt_w=avail*0.575f, obj_w=avail-jnt_w-8.0f;

        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.118f,0.126f,0.157f,1.0f});
        ImGui::BeginChild("##jnts",{jnt_w,bot_h},true,NO_SCROLL);
        sec("GELENKE  (live)");
        ImGui::BeginDisabled(!rdy);
        {
            const float sw=jnt_w-102.0f;
            std::lock_guard<std::mutex> lk(g_jv_mx);
            if(robot==0){
                const char*jn[]={"J1","J2","J3","J4","J5","J6"};
                for(int i=0;i<6;i++){
                    if(!arm_drag[i]) arm_edit[i]=g_jv_arm[i];
                    std::string lbl=std::string(jn[i])+"##lv";
                    auto st=joint_slider(lbl.c_str(),&arm_edit[i],-3.14159f,3.14159f,sw);
                    arm_drag[i]=st.active;
                    if(st.done){
                        float v=arm_edit[i]; int idx=i;
                        std::thread([v,idx]{
                            if(!g_arm_mg)return;
                            auto jv=g_arm_mg->getCurrentJointValues();
                            if(idx<(int)jv.size())jv[idx]=v;
                            move_joints(g_arm_mg,jv,"Slider");
                        }).detach();
                    }
                }
            } else {
                const char*jn[]={"J1","J2","J3","J4"};
                float lo[]={-3.14159f,-3.14159f,-0.40f,-3.14159f};
                float hi[]={ 3.14159f, 3.14159f, 0.00f, 3.14159f};
                for(int i=0;i<4;i++){
                    if(!scara_drag[i]) scara_edit[i]=g_jv_scara[i];
                    std::string lbl=std::string(jn[i])+"##lv";
                    auto st=joint_slider(lbl.c_str(),&scara_edit[i],lo[i],hi[i],sw,i==2);
                    scara_drag[i]=st.active;
                    if(st.done){
                        float v=scara_edit[i]; int idx=i;
                        std::thread([v,idx]{
                            if(!g_scara_mg)return;
                            auto jv=g_scara_mg->getCurrentJointValues();
                            if(idx<(int)jv.size())jv[idx]=v;
                            move_joints(g_scara_mg,jv,"Slider");
                        }).detach();
                    }
                }
            }
        }
        ImGui::EndDisabled();
        ImGui::EndChild(); ImGui::PopStyleColor();
        ImGui::SameLine(0,8);

        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.118f,0.126f,0.157f,1.0f});
        ImGui::BeginChild("##objs",{obj_w,bot_h},true,NO_SCROLL);
        sec("SIMULATION");
        {std::lock_guard<std::mutex> lk(g_obj_mx);
         for(auto&[name,pos]:g_obj_pos){
             bool pk=g_picking&&std::any_of(g_picked.begin(),g_picked.end(),[&](auto&po){return po.name==name;});
             if(pk) ImGui::TextColored(COL_WARN,"* %-14s %+.3f %+.3f %+.3f",name.c_str(),pos[0],pos[1],pos[2]);
             else   ImGui::TextColored(COL_DIM, "  %-14s %+.3f %+.3f %+.3f",name.c_str(),pos[0],pos[1],pos[2]);
         }}
        ImGui::EndChild(); ImGui::PopStyleColor();
        ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();

        // ── LOG PANEL (full width) ────────────────────────────────────
        ImGui::PushStyleColor(ImGuiCol_ChildBg,{0.118f,0.126f,0.157f,1.0f});
        ImGui::BeginChild("##logpanel",{0,128},true);
        sec("LOG");
        ImGui::BeginChild("##logscroll",{0,0},false);
        {
            std::lock_guard<std::mutex> lk(g_log_mx);
            for(auto&line:g_log){
                bool is_err=line.find("fehlgeschlagen")!=std::string::npos
                           ||line.find("FAILED")!=std::string::npos
                           ||line.find("E-STOP")!=std::string::npos;
                bool is_ok =line.find(" OK")!=std::string::npos
                           ||line.find("abgeschlossen")!=std::string::npos
                           ||line.find("Bereit")!=std::string::npos
                           ||line.find("Gespeichert")!=std::string::npos;
                if(is_err)      ImGui::TextColored(COL_RED, "%s",line.c_str());
                else if(is_ok)  ImGui::TextColored(COL_GREEN,"%s",line.c_str());
                else            ImGui::TextColored(COL_DIM,  "%s",line.c_str());
            }
            size_t cur_sz=g_log.size();
            if(cur_sz>prev_log_sz) ImGui::SetScrollHereY(1.0f);
            prev_log_sz=cur_sz;
        }
        ImGui::EndChild();
        ImGui::EndChild(); ImGui::PopStyleColor();

        ImGui::End();
        ImGui::Render();
        int fw,fh; glfwGetFramebufferSize(win,&fw,&fh);
        glViewport(0,0,fw,fh);
        glClearColor(0.055f,0.059f,0.075f,1.0f);  // slightly darker than window for depth
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(win);
    }

    g_picking=false; g_ros_running=false; g_seq_stop_req=true;
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext();
    glfwDestroyWindow(win); glfwTerminate();
    full_shutdown();
    return 0;
}