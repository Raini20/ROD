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

| Package | Beschreibung | Status |
|---|---|---|
| `robot_arm_6dof_assembly` | URDF, Meshes, ros2_control Config und gz.launch.py für den Knickarm | aktiv |
| `arm_moveit` | MoveIt2 Konfiguration (SRDF, Kinematics, Controller, RViz) für den Knickarm | aktiv |
| `scara_4` | URDF, Meshes, ros2_control Config und gz.launch.py für den SCARA | aktiv |
| `scara_moveit` | MoveIt2 Konfiguration für den SCARA | aktiv |
| `rod_scene` | Szenenobjekte als GLB-Meshes (Säulen, Fixiereinheit, Förderbänder, Werkstücke) | aktiv |
| `rod_cell` | Kombinierte Launch Files für die gesamte Zelle | aktiv |
| `rod_demo` | C++ Demo Scripts — vordefinierte Posen anfahren (Arm + SCARA) | aktiv |
| `rod_hmi` | Dear ImGui HMI — TCP-Steuerung, Pose speichern/exportieren, Sequenz | aktiv |
| `SolidWorks` | Originale SolidWorks-Exports (COLCON_IGNORE, nur Referenz) | Archiv |
| `EasyBot` | Referenzprojekt vom Lektor (COLCON_IGNORE) | Referenz |

---

## Setup

### Voraussetzungen

```bash
sudo apt install ros-jazzy-moveit ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-state-publisher* \
  libglfw3-dev libglfw3
```

### Repository klonen

```bash
mkdir -p ~/rod_ws/src
cd ~/rod_ws/src
git clone https://github.com/Raini20/ROD.git .
```

### Workspace bauen

```bash
cd ~/rod_ws && colcon build && source install/setup.bash
```

---

## Starten

### A – Gesamte Zelle (visuell, beide Roboter statisch)

Für Screenshots und Präsentationen — Roboter sind statisch (keine Physik).

```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch rod_cell cell_visual.launch.py
```

### B – HMI (empfohlen — startet alles automatisch)

Ein einziger Befehl öffnet das GUI und startet Gazebo + beide MoveGroups automatisch im Hintergrund.

```bash
cd ~/rod_ws && source install/setup.bash
ros2 run rod_hmi rod_hmi
```

**Funktionen im HMI:**
- Roboter-Auswahl (Arm / SCARA)
- TCP-Steuerung mit X+/X-/Y+/Y-/Z+/Z- Buttons
- Schrittweite per Slider einstellbar (0.5cm – 20cm)
- Aktuelle TCP-Position anzeige
- Pose speichern (mit Name)
- Gespeicherte Sequenz ausführen
- Posen exportieren als CSV (`/tmp/rod_poses.csv`)

### C – Beide Roboter mit MoveIt + Gazebo (manuell, 5 Terminals)

**Terminal 1 – Gazebo + beide Controller:**
```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch rod_cell cell.launch.py
```

**Terminal 2 – MoveGroup Knickarm:**
```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch arm_moveit move_group.launch.py
```

**Terminal 3 – MoveGroup SCARA:**
```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch scara_moveit move_group.launch.py
```

**Terminal 4 – RViz Knickarm:**
```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch arm_moveit moveit_rviz.launch.py
```

**Terminal 5 – RViz SCARA:**
```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch scara_moveit moveit_rviz.launch.py
```

### D – Demo Poses (vordefinierte Posen abfahren)

Benötigt: Terminal C1 + C2 + C3 laufen bereits.

```bash
# Knickarm Demo
cd ~/rod_ws && source install/setup.bash
ros2 launch rod_demo arm_demo.launch.py

# SCARA Demo
cd ~/rod_ws && source install/setup.bash
ros2 run rod_demo scara_demo
```

### E – Nur RViz (ohne Gazebo, für schnelles Testen)

```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch arm_moveit demo.launch.py
```

```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch scara_moveit demo.launch.py
```

---

## Status & TODOs

### ✅ Erledigt
- 6-DOF Knickarm URDF mit Saugnapf-Endeffektor
- SCARA URDF mit Endeffektor (Merle)
- MoveIt2 Konfiguration für beide Roboter
- Gazebo Harmonic Integration — Knickarm planbar und ausführbar
- Gazebo Harmonic Integration — SCARA planbar und ausführbar
- Beide Roboter in einem Gazebo mit namespaced Controller Managern
- Zellenszene: Säulen, Fixiereinheit, Förderbänder, Werkstücke (GLB)
- Visualisierungs-Launch (`cell_visual.launch.py`)
- Demo Scripts — Knickarm + SCARA fahren vordefinierte Posen (Joint + Kartesisch mit Quaternionen)
- HMI — Dear ImGui GUI mit TCP-Steuerung, Pose speichern/exportieren, Sequenz ausführen
- Startskript — HMI startet alles automatisch (ein Befehl)
- ROS2-Package Struktur

### ❌ Must-Have TODOs (Pflicht laut Angabe)
- [ ] **Dokumentation als PDF** — Anwendungsfall beschreiben, alle Pakete/Abhängigkeiten dokumentieren, Startanleitung

### 💡 Nice-to-Have TODOs
- [ ] Sequenz-Fix im HMI (Planung schlägt manchmal fehl)
- [ ] Objekte greifen (gazebo_ros_link_attacher)
- [ ] Schutzzaun in der Szene
- [ ] Backup-Video der Simulation für Präsentation
- [ ] Beide Roboter führen den Ablauf automatisch aus

---

## Branches

| Branch | Inhalt |
|---|---|
| `main` | Stabiler Stand — beide Roboter, Szene, MoveIt |
| `demo_poses` | rod_demo C++ Package — vordefinierte Posen mit Quaternionen |
| `hmi` | rod_hmi Dear ImGui HMI (in Entwicklung) |
