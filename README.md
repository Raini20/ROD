# ROD – Robot Cell Simulation

ROS2 Jazzy | Gazebo Harmonic | MoveIt2  
FH Technikum Wien – Kompetenzfeld Digital Manufacturing, Automation & Robotics

## Übersicht

Simulation einer industriellen Roboterzelle mit zwei Robotern:
- **6-DOF Knickarm** (kinematisch angelehnt an UR15) mit Saugnapf-Endeffektor
- **SCARA** (4-DOF) mit Schraubwerkzeug

**Usecase:** Batteriegehäuse-Montage  
1. Knickarm greift Gehäuse von FB1 → legt in Fixiereinheit
2. SCARA verschraubt Gehäuse
3. Knickarm greift Deckel von FB2 → legt auf Gehäuse
4. SCARA verschraubt Deckel (4 Schrauben)
5. Knickarm transferiert fertige Assembly auf FB3

---

## Packages

| Package | Beschreibung |
|---|---|
| `robot_arm_6dof_assembly` | URDF + Meshes + Launch für den Knickarm |
| `arm_moveit` | MoveIt2 Konfiguration für den Knickarm |
| `scara_4` | URDF + Meshes + Launch für den SCARA |
| `scara_moveit` | MoveIt2 Konfiguration für den SCARA |
| `rod_scene` | Szenenobjekte (Säulen, Fixiereinheit, Förderbänder, Werkstücke) |
| `rod_cell` | Kombiniertes Launch File für die gesamte Zelle |

---

## Setup

### Voraussetzungen

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-publisher*
```

### Repository klonen

```bash
mkdir -p ~/rod_ws/src
cd ~/rod_ws/src
git clone https://github.com/Raini20/ROD.git .
```

### Workspace bauen

```bash
cd ~/rod_ws
colcon build
source install/setup.bash
```

---

## Starten

### A Gesamte Zelle (visuell, beide Roboter statisch)

```bash
ros2 launch rod_cell cell.launch.py
```

### B Knickarm mit MoveIt + Gazebo (voll steuerbar)

**Terminal 1 – Gazebo + Controller:**
```bash
ros2 launch robot_arm_6dof_assembly gz.launch.py
```

**Terminal 2 – MoveGroup:**
```bash
ros2 launch arm_moveit move_group.launch.py
```

**Terminal 3 – RViz:**
```bash
ros2 launch arm_moveit moveit_rviz.launch.py
```

### C Nur RViz (ohne Gazebo)

```bash
ros2 launch arm_moveit demo.launch.py
ros2 launch scara_moveit demo.launch.py
```

---

---

## Status & TODOs

### ✅ Erledigt
- 6-DOF Knickarm URDF mit Saugnapf-Endeffektor
- SCARA URDF mit Endeffektor (Merle)
- MoveIt2 Konfiguration für beide Roboter
- Gazebo Harmonic Integration — Knickarm planbar und ausführbar
- Zellenszene: Säulen, Fixiereinheit, Förderbänder, Werkstücke (GLB)
- Visualisierungs-Launch (`cell.launch.py`)
- ROS2-Package Struktur

### ❌ Must-Have TODOs (Pflicht laut Angabe)
- [ ] **SCARA in Gazebo steuerbar** — MoveIt + Gazebo für SCARA analog zum Arm aufsetzen
- [ ] **HMI** — mindestens Konsolenapplikation die einen Roboter am TCP linear bewegen kann (IK nötig)
- [ ] **Startskript** — ein einziger Befehl (bash oder ros2 launch) startet die gesamte Simulation
- [ ] **Dokumentation als PDF** — Anwendungsfall beschreiben, alle Pakete/Abhängigkeiten dokumentieren, Startanleitung

### 💡 Nice-to-Have TODOs
- [ ] GUI für HMI (z.B. Tkinter oder Qt)
- [ ] Schutzzaun in der Szene
- [ ] Backup-Video der Simulation für Präsentation
- [ ] Beide Roboter führen den Ablauf automatisch aus (Programmierung des Workflows)