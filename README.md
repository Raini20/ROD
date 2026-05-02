# ROD Workspace Setup

## 1. ROS2 Jazzy installieren
Falls noch nicht installiert:
https://docs.ros.org/en/jazzy/Installation.html

---

## 2. MoveIt installieren
```bash
sudo apt install ros-jazzy-moveit
```

---

## 3. Workspace anlegen
```bash
mkdir -p ~/rod_ws/src
cd ~/rod_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

---

## 4. Repo klonen
```bash
cd ~/rod_ws/src
git clone https://github.com/Raini20/ROD.git .
```

---

## 5. Bauen
```bash
cd ~/rod_ws
colcon build
source install/setup.bash
```

---

## 6. Testen — 6DOF Arm in RViz
```bash
source ~/rod_ws/install/setup.bash
ros2 launch robot_arm_6dof_assembly launch_robot.py
```

## 7. Testen — SCARA in RViz
```bash
source ~/rod_ws/install/setup.bash
ros2 launch scara_4 launch_scara.py
```

---

## Workspace sourcen (jeden Terminal neu)
```bash
source /opt/ros/jazzy/setup.bash
source ~/rod_ws/install/setup.bash
```

Tipp: In `~/.bashrc` eintragen damit es automatisch passiert:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/rod_ws/install/setup.bash" >> ~/.bashrc
```
