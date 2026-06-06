# ROD – Robot Cell Simulation

ROS2 Jazzy | Gazebo Harmonic | MoveIt2  
FH Technikum Wien – Kompetenzfeld Digital Manufacturing, Automation & Robotics

## Übersicht

Simulation einer industriellen Roboterzelle mit zwei Robotern:
- **6-DOF Knickarm** (kinematisch angelehnt an UR15) mit Saugnapf-Endeffektor
- **SCARA** (4-DOF) mit Schraubwerkzeug

**Usecase:** Toaster-Gehäuse-Montage  
1. Knickarm greift Gehäuse von Förderband → legt in Fixiereinheit
2. Knickarm greift Deckel → legt auf Gehäuse
3. SCARA verschraubt Deckel
4. Knickarm transferiert fertige Assembly auf Ausgangs-Förderband

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
| `rod_hmi` | Dear ImGui HMI — TCP-Steuerung, Pose-Verwaltung, Sequenz, CSV Import/Export | aktiv |
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
git checkout pick_place
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

### D – Nur RViz (ohne Gazebo, für schnelles Testen)

```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch arm_moveit demo.launch.py
```

```bash
cd ~/rod_ws && source install/setup.bash
ros2 launch scara_moveit demo.launch.py
```

---

## HMI – Bedienung

Das HMI (`rod_hmi`) ist die primäre Schnittstelle zur Simulation. Es startet Gazebo und beide MoveGroups automatisch beim Start.

### TCP-Steuerung
- X+/X−/Y+/Y−/Z+/Z− Buttons für kartesische Bewegung
- Schrittweite per Slider einstellbar (0.5 cm – 20 cm)
- TCP Rotation Roll/Pitch/Yaw (nur Knickarm)

### Gelenkswinkel (absolut)
Slider zum manuellen Setzen von Zielwinkeln. Bewegung wird mit **"Joints anfahren"** ausgelöst.

### Gelenkswinkel (live)
Zweiter Slider-Satz der die **aktuelle Gelenkposition** in Echtzeit anzeigt (Aktualisierung alle 200 ms).  
Slider ziehen und **loslassen** → Roboter fährt direkt auf diesen Winkel, ohne extra Button.

### Pose-Verwaltung

**Speichern:**  
Name eingeben → Aktion wählen (Pick/Place bzw. Screw/Unscrew) → **"Speichern"**  
Die aktuelle TCP-Pose und die Gelenkkonfiguration (Joint-Werte) werden gemeinsam gespeichert.

**Gespeicherte Posen Tabelle:**  
Zeigt alle Posen mit Index, Roboter, Name, Position, Aktion und Konfigurations-Status (OK / –).

**Konfiguration wählen:**  
Pose aus Dropdown wählen und:
- **"Anfahren"** — fährt die Pose an (MoveIt sucht eine IK-Lösung)
- **"Konfig übernehmen"** — speichert die aktuelle Gelenkkonfiguration für diese Pose
- **"Elbow Flip"** — spiegelt Joint 3 um 180° (wechselt zwischen Elbow-Up / Elbow-Down)

Grünes **"Konfig OK"** = Joint-Werte gespeichert → Sequenz verwendet diese exakte Konfiguration.

### Sequenz
**"Sequenz ausführen"** fährt alle gespeicherten Posen der Reihe nach ab.  
Pro Schritt wird zuerst ein kartesischer Pfad versucht (direkte Linie). Falls dieser scheitert (fraction ≤ 0.5), wird mit den gespeicherten Joint-Werten geplant — so wird immer die korrekte Arm-Konfiguration verwendet.

### CSV Import / Export

**Format:**
```
# ROD Pose Export
# robot, name, x, y, z, qx, qy, qz, qw, action, j0, j1, ...
arm, Home, 0.150, 0.205, 0.755, 0.707, 0.0, 0.707, 0.0, None, 0.0, -0.5, 1.2, 0.0, 0.5, 0.0
```

- Pfad im Textfeld einstellbar (Standard: `~/rod_ws/src/rod_hmi/rod_poses.csv`)
- **"Exportieren"** — schreibt alle aktuellen Posen inkl. Joint-Werte
- **"Importieren"** — lädt Posen aus CSV, hängt sie an die bestehende Liste an
- Rückwärtskompatibel: alte CSVs ohne Joint-Spalten werden korrekt eingelesen (Konfig = leer)

---

## Status & TODOs

### ✅ Vollständig implementiert
- 6-DOF Knickarm URDF mit Saugnapf-Endeffektor
- SCARA URDF mit Endeffektor
- MoveIt2 Konfiguration für beide Roboter (SRDF, Kinematics, Controller)
- Gazebo Harmonic — beide Roboter planbar und ausführbar in einer Simulation
- Namespaced Controller Manager (`/arm`, `/scara`)
- Zellenszene: Säulen, Fixiereinheit, Förderbänder, Werkstücke (GLB)
- HMI startet alles automatisch (`ros2 run rod_hmi rod_hmi`)
- TCP-Steuerung (Translation + Rotation)
- Joint-Slider (absolut + live)
- Pose speichern mit automatischer Joint-Konfiguration
- Konfigurationsauswahl mit Anfahren / Konfig übernehmen / Elbow Flip
- Sequenz ausführen mit kartesischem Pfad + Joint-Fallback
- CSV Import + Export mit Joint-Werten, konfigurierbarer Pfad

### 🟡 Teilweise implementiert
- Pick / Place → Greifer-Simulation aktiv, link_attacher noch nicht integriert
- Screw / Unscrew → Platzhalter, SCARA joint_4 Rotation noch nicht implementiert

### ❌ Noch offen (Pflicht laut Angabe)
- [ ] Vollautomatischer Ablauf (Knickarm + SCARA kombinierter Workflow)
- [ ] Dokumentation als PDF

### 💡 Nice-to-Have
- [ ] link_attacher Integration (echtes Pick & Place)
- [ ] Schrauben-Simulation (SCARA joint_4 Rotation)
- [ ] Schutzzaun in der Szene
- [ ] Backup-Video der Simulation

---

## Branches

| Branch | Inhalt |
|---|---|
| `main` | Stabiler Basisstand — beide Roboter, Szene, MoveIt2 Konfiguration |
| `pick_place` | Aktiver Entwicklungsbranch — HMI, Sequenz, CSV Import/Export, Pick & Place |
