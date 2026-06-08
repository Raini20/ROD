# ROD – Robot Cell Simulation
 
ROS2 Jazzy | Gazebo Harmonic | MoveIt2  
FH Technikum Wien – Kompetenzfeld Digital Manufacturing, Automation & Robotics
 
## Übersicht
 
Simulation einer industriellen Roboterzelle mit zwei Robotern:
- **6-DOF Knickarm** (kinematisch angelehnt an UR15) mit Saugnapf-Endeffektor
- **SCARA** (4-DOF) mit Schraubwerkzeug
**Usecase:** Toaster-Gehäuse-Montage
1. Knickarm greift Gehäuse (`toaster_shell`) vom Förderband → legt in Fixiereinheit
2. Knickarm greift Deckel (`toaster_innen`) + 4 Schrauben gleichzeitig → legt auf Gehäuse
3. SCARA verschraubt Deckel (4 Schrauben)
4. Knickarm greift fertige Assembly (alle 6 Teile) → transferiert auf Ausgangs-Förderband
---
 
## Packages
 
| Package | Beschreibung | Status |
|---|---|---|
| `robot_arm_6dof_assembly` | URDF, Meshes, ros2_control Config für den Knickarm | aktiv |
| `arm_moveit` | MoveIt2 Konfiguration (SRDF, Kinematics, Controller, RViz) für den Knickarm | aktiv |
| `scara_4` | URDF, Meshes, ros2_control Config für den SCARA | aktiv |
| `scara_moveit` | MoveIt2 Konfiguration für den SCARA | aktiv |
| `rod_scene` | Szenenobjekte als GLB-Meshes (Säulen, Fixiereinheit, Förderbänder, Werkstücke, Schrauben) | aktiv |
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
 
### A – HMI (empfohlen — startet alles automatisch)
 
Ein einziger Befehl öffnet das GUI und startet Gazebo + beide MoveGroups automatisch im Hintergrund.
 
```bash
cd ~/rod_ws && source install/setup.bash
ros2 run rod_hmi rod_hmi
```
 
### B – Beide Roboter mit MoveIt + Gazebo (manuell, 5 Terminals)
 
**Terminal 1 – Gazebo + beide Controller:**
```bash
ros2 launch rod_cell cell.launch.py
```
**Terminal 2 – MoveGroup Knickarm:**
```bash
ros2 launch arm_moveit move_group.launch.py
```
**Terminal 3 – MoveGroup SCARA:**
```bash
ros2 launch scara_moveit move_group.launch.py
```
**Terminal 4 – RViz Knickarm:**
```bash
ros2 launch arm_moveit moveit_rviz.launch.py
```
**Terminal 5 – RViz SCARA:**
```bash
ros2 launch scara_moveit moveit_rviz.launch.py
```
 
### C – Nur RViz (ohne Gazebo, für schnelles Testen)
 
```bash
ros2 launch arm_moveit demo.launch.py
ros2 launch scara_moveit demo.launch.py
```
 
---
 
## HMI – Bedienung
 
Das HMI (`rod_hmi`) ist die primäre Schnittstelle zur Simulation. Es startet Gazebo und beide MoveGroups automatisch beim Start (~25s Wartezeit).
 
### TCP-Steuerung
- X+/X−/Y+/Y−/Z+/Z− Buttons für kartesische Bewegung
- Schrittweite per Slider einstellbar (0.5 cm – 20 cm)
- TCP Rotation Roll/Pitch/Yaw (nur Knickarm)
### PICK / PLACE / SCREW / UNSCREW
Manuelle Buttons zum direkten Auslösen einer Pick- oder Place-Aktion.
- **PICK / SCREW** — greift alle Objekte innerhalb von 300 mm Radius gleichzeitig
- **PLACE / UNSCREW** — setzt alle gegriffenen Objekte ab
- **Scene Reset** — setzt alle Simulationsobjekte auf ihre Startpositionen zurück
### Gelenkswinkel (absolut)
Slider zum manuellen Setzen von Zielwinkeln. Bewegung wird mit **"Joints anfahren"** ausgelöst.
 
### Gelenkswinkel (live)
Zweiter Slider-Satz der die **aktuelle Gelenkposition** in Echtzeit anzeigt (Aktualisierung alle 200 ms).  
Slider ziehen und **loslassen** → Roboter fährt direkt auf diesen Winkel, ohne extra Button.
 
### Pose-Verwaltung
 
**Speichern:**  
Name eingeben → Aktion wählen (Pick/Place bzw. Screw/Unscrew) → **"Speichern"**  
TCP-Pose und Gelenkkonfiguration (Joint-Werte) werden automatisch gemeinsam gespeichert.  
Der Namensvorschlag zählt automatisch hoch wenn der Name mit einer Zahl endet (z.B. `Pose_1` → `Pose_2`).
 
**Gespeicherte Posen Tabelle:**  
Zeigt alle Posen mit Index, Roboter, Name, Position, Aktion und Konfigurations-Status (OK / –).  
Zeile anklicken → Pose wird automatisch im Dropdown ausgewählt.
 
**Konfiguration wählen:**  
Pose aus Dropdown wählen (oder Tabellenzeile klicken) und:
- **"Anfahren"** — fährt die Pose an (MoveIt sucht eine IK-Lösung)
- **"Konfig übernehmen"** — speichert die aktuelle Gelenkkonfiguration für diese Pose
- **"Elbow Flip"** — spiegelt Joint 3 um 180° (wechselt zwischen Elbow-Up / Elbow-Down) ⚠️ *experimentell, funktioniert nicht zuverlässig*
Grünes **"Konfig OK"** = Joint-Werte gespeichert → Sequenz verwendet diese exakte Konfiguration.
 
### Sequenz
**"Sequenz ausführen"** fährt alle gespeicherten Posen der Reihe nach ab.  
Pro Schritt:
1. Kartesischer Pfad versucht (direkte TCP-Linie)
2. Falls fraction ≤ 0.5 → Fallback auf gespeicherte Joint-Werte
3. Bei Pick/Place Aktionen: 500 ms Wartezeit vor Ausführung
**"Gewählte löschen"** — löscht nur die aktuell im Dropdown gewählte Pose.
 
### CSV Import / Export
 
**Format:**
```
# ROD Pose Export
# robot, name, x, y, z, qx, qy, qz, qw, action, j0, j1, ...
arm, Home, 0.150, 0.205, 0.755, 0.707, 0.0, 0.707, 0.0, None, -0.44, -0.04, -1.53, -1.57, 2.70, -1.57
```
 
- Pfad im Textfeld einstellbar (Standard: `~/rod_ws/src/rod_hmi/rod_poses.csv`)
- **"Exportieren"** — schreibt alle aktuellen Posen inkl. Joint-Werte
- **"Importieren"** — lädt Posen aus CSV, hängt sie an die bestehende Liste an
- Rückwärtskompatibel: alte CSVs ohne Joint-Spalten werden korrekt eingelesen
---
 
## Multi-Object Pick
 
Der Pick-Mechanismus greift automatisch alle Objekte innerhalb eines **300 mm Radius** um den TCP:
 
| Schritt | Gegriffene Objekte |
|---|---|
| Shell holen | `toaster_shell` (1 Objekt) |
| Deckel holen | `toaster_innen` + 4 Schrauben (5 Objekte) |
| Assembly transferieren | `toaster_shell` + `toaster_innen` + 4 Schrauben (6 Objekte) |
 
Alle Objekte folgen dem TCP mit korrekter relativer Position und Orientierung.
 
---
 
## Status & TODOs
 
### ✅ Vollständig implementiert
- 6-DOF Knickarm URDF + SCARA URDF mit Endeffektoren ⚠️ *TCP-Offset des Werkzeugs (Saugnapf) nicht korrekt in Yaw Achse*
- MoveIt2 Konfiguration für beide Roboter
- Gazebo Harmonic — beide Roboter in einer Simulation, namespaced Controller
- Zellenszene: Säulen, Fixiereinheit, Förderbänder, Toasterteile, 4 Schrauben
- HMI startet alles automatisch
- TCP-Steuerung (Translation + Rotation)
- Joint-Slider (absolut + live mit Direktsteuerung)
- Pose speichern mit automatischer Joint-Konfiguration + Auto-Nummerierung
- Konfigurationsauswahl mit Anfahren / Konfig übernehmen / Elbow Flip
- Klickbare Pose-Tabelle
- Sequenz mit kartesischem Pfad + Joint-Fallback
- Multi-Object Pick (bis zu 6 Objekte gleichzeitig, radius-basiert)
- PICK / PLACE / SCREW / UNSCREW manuelle Buttons
- Scene Reset
- CSV Import + Export mit Joint-Werten, konfigurierbarer Pfad
### 🟡 Teilweise implementiert
- Pick/Place — Greifer-Simulation aktiv (visuelle Verfolgung), kein physischer link_attacher
- Screw/Unscrew — Aktionen definiert, SCARA joint_4 Rotation noch nicht animiert
### ❌ Noch offen
- [ ] Vollautomatischer Ablauf (Knickarm + SCARA kombinierter Workflow)
- [ ] Dokumentation als PDF
### 💡 Nice-to-Have
- [ ] Gelenkswinkel-Anzeige in Grad statt Radiant (siehe `grad_umstellung.md`)
- [ ] Schutzzaun in der Szene
- [ ] Backup-Video der Simulation
---
 
## Branches
 
| Branch | Inhalt |
|---|---|
| `main` | Stabiler Basisstand — beide Roboter, Szene, MoveIt2 Konfiguration |
| `pick_place` | Aktiver Entwicklungsbranch — HMI, Sequenz, CSV, Multi-Object Pick & Place |
 