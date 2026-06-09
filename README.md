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
3. SCARA verschraubt Deckel (4 Schrauben einzeln)
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

Das HMI (`rod_hmi`) ist die primäre Schnittstelle zur Simulation. Es startet Gazebo und beide MoveGroups automatisch beim Start (~25 s Wartezeit).

### Headerzeile

- **E-STOP** (rot, oben rechts) — sofortiger Nothalt, stoppt beide Roboter via `mg->stop()`. Alle Bewegungsbuttons werden deaktiviert. Mit **"E-Stop Reset"** wird der Zustand zurückgesetzt.
- **Quit** (grau, rechts neben E-STOP) — beendet das HMI sauber: stoppt zuerst alle Hintergrundprozesse (Gazebo, beide MoveGroups, ros2_control, robot_state_publisher) per SIGTERM, wartet kurz, schickt SIGKILL an hartnäckige Prozesse und setzt den ROS2 Daemon zurück — damit der nächste Start eine saubere Umgebung vorfindet.
- **Geschwindigkeit** (Slider, 5–100 %) — Geschwindigkeits-Override für alle Bewegungen beider Roboter, wird live angewendet.
- **TCP-Anzeige** (rechts) — aktuelle TCP-Position des gewählten Roboters in Weltkoordinaten.
- **Statuszeile** — letzte HMI-Meldung; vollständiges Log im Log-Panel unten.

### TCP-Steuerung

- X+/X−/Y+/Y−/Z+/Z− Buttons für kartesische Bewegung
- Schrittweite per Slider einstellbar (0,5 cm – 20 cm), separat für XY und Z
- TCP Rotation Roll/Pitch/Yaw (nur Knickarm), Schrittweite in Rad + Grad angezeigt
- **Home** — fährt den gewählten Roboter in die im SRDF definierte `home`-Pose

### PICK / PLACE / SCREW / UNSCREW / Scene Reset

Manuelle Buttons zum direkten Auslösen einer Aktion.

- **PICK** (Knickarm) — greift alle Objekte innerhalb von 300 mm Radius gleichzeitig
- **PLACE** (Knickarm) — setzt alle gegriffenen Objekte ab
- **SCREW** (SCARA) — greift die nächste Schraube (bevorzugt `schraube_*`, exakt 1 Objekt), fährt 20 mm runter, dreht J4 um 2 Umdrehungen, setzt ab, fährt zurück
- **UNSCREW** — Platzhalter
- **Scene Reset** — setzt alle Simulationsobjekte auf ihre exakten Startpositionen und -orientierungen zurück

### Gelenkswinkel (live)

Slider zeigen die **aktuelle Gelenkposition** in Echtzeit (Aktualisierung alle 200 ms).  
Slider ziehen und **loslassen** → Roboter fährt direkt auf diesen Winkel, ohne extra Button.  
Winkelanzeige: Radiant im Slider + Grad als Label (z. B. `−80,3°`).  
J3 des SCARA (Linearachse) wird in mm angezeigt (−400 mm bis 0 mm).

### Pose-Verwaltung

**Speichern:**  
Name eingeben → Aktion wählen → **"Speichern"**  
TCP-Pose und Gelenkkonfiguration werden automatisch gemeinsam gespeichert.  
Der Namensvorschlag zählt automatisch hoch (z. B. `Pose_1` → `Pose_2`).

**Bearbeiten:**  
Tabellenzeile anklicken → Name und Aktion werden in die Felder übernommen.  
**"Aendern"** — aktualisiert Name/Aktion der gewählten Pose ohne die Position neu zu teachen.

**Aktionen im Dropdown:**

| Roboter | Optionen |
|---|---|
| Knickarm | `--`, `Pick`, `Place`, `Reset` |
| SCARA | `--`, `Screw`, `Unscrew`, `Reset` |

`Reset` führt einen Scene Reset als Teil der Sequenz aus — sinnvoll als letzter Schritt im Loop.

**Gespeicherte Posen Tabelle:**  
Zeigt alle Posen mit Index, Roboter, Name, Position, Aktion und Konfigurations-Status (`ok` / `–`).  
Zeile anklicken → Pose wird in den Feldern und im Dropdown ausgewählt.  
**↑ / ↓** — Reihenfolge der Einträge ändern.  
**Duplikat** — klont die gewählte Pose mit `_2`-Suffix.

**Konfiguration:**  
- **Anfahren** — fährt die gewählte Pose an (bevorzugt gespeicherte Joint-Werte)
- **Konfig** — speichert die aktuelle Gelenkkonfiguration für diese Pose nach
- **Elbow** — spiegelt Joint 3 um 180° (Elbow-Up / Elbow-Down) ⚠️ *experimentell*

Grünes **`ok`** = Joint-Werte gespeichert → Sequenz verwendet diese exakte Konfiguration.

### Sequenz

| Button | Funktion |
|---|---|
| **Starten** | Führt alle Posen der Reihe nach aus |
| **Pause / Resume** | Hält nach der aktuellen Bewegung an, setzt fort |
| **Stop** | Bricht die Sequenz ab und stoppt den Roboter sofort |
| **Loop** | Wiederholt die Sequenz endlos (Zähler rechts daneben) |

Der aktuell ausgeführte Schritt wird in der Tabelle amber hinterlegt.

Pro Schritt:
1. Gespeicherte Joint-Werte werden bevorzugt (deterministische Konfiguration)
2. Falls keine vorhanden → kartesischer Pfad (fraction > 0,5)
3. Falls auch das scheitert → MoveIt Pose-Planner als Fallback

**Loop + Scene Reset:** `Reset`-Aktion ans Ende der Sequenz hängen → nach jedem Durchlauf werden alle Objekte in die Ausgangslage zurückgesetzt.

### CSV Import / Export

**Format:**
```
# ROD Pose Export
# robot,name,x,y,z,qx,qy,qz,qw,action,j0,j1,...
arm,Home,0.150,0.205,0.755,0.707,0.0,0.707,0.0,None,-0.44,-0.04,-1.53,-1.57,2.70,-1.57
```

- Pfad im Textfeld einstellbar (Standard: `~/rod_ws/src/rod_hmi/rod_poses.csv`)
- **Export** — schreibt alle aktuellen Posen inkl. Joint-Werte
- **Import** — lädt Posen aus CSV, hängt sie an die bestehende Liste an
- Rückwärtskompatibel: alte CSVs ohne Joint-Spalten werden korrekt eingelesen

### Log-Panel

Scrollbares Protokoll aller Statusmeldungen mit Timestamp (`HH:MM:SS`).  
Farbkodierung: Rot = Fehler, Grün = Erfolg, Grau = Info.  
Scrollt automatisch auf den neuesten Eintrag.

---

## Multi-Object Pick

Der Pick-Mechanismus greift automatisch alle Objekte innerhalb eines **300 mm Radius** um den TCP:

| Schritt | Gegriffene Objekte |
|---|---|
| Shell holen | `toaster_shell` (1 Objekt) |
| Deckel + Schrauben holen | `toaster_innen` + 4 × `schraube_*` (5 Objekte) |
| Assembly transferieren | alle 6 Teile |

Alle Objekte folgen dem TCP mit korrekter relativer Position und Orientierung.

Der SCARA-SCREW greift **exakt 1 Objekt** (nächste Schraube im 300 mm Radius, bevorzugt `schraube_*`).

---

## Technische Details

### Pick-Implementierung

`do_pick_impl(mg, base_ox, base_oy, base_oz, prefer_substr, max_count)`:
- `prefer_substr` — Objekte deren Name diesen String enthält werden bevorzugt
- `max_count` — maximale Anzahl zu greifender Objekte (Arm: 999, SCARA-Screw: 1)
- Objekte werden nach Präferenz, dann nach Distanz sortiert

### IK-Konfiguration

`computeCartesianPath` allein wählt Elbow-Konfiguration frei → inkonsistente Bewegungen.  
**Fix:** Posen werden mit `getCurrentJointValues()` gespeichert. Die Sequenz setzt `setJointValueTarget()` mit diesen Werten → deterministische Wiederholung.

### Roboter-Weltrahmen-Offsets

Aus `cell.launch.py` (`-x -y -z` Spawn-Argumente):

| Roboter | X | Y | Z |
|---|---|---|---|
| Knickarm | −0,75 m | 0,00 m | 1,00 m |
| SCARA | +1,00 m | 0,00 m | 1,00 m |

### Scene Reset

Stellt **Position und Orientierung** aller Objekte aus `k_init_pos` / `k_init_ori` wieder her.  
(Bug in früherer Version: Orientierung wurde aus dem veränderlichen `g_obj_ori` gelesen statt aus dem konstanten Initialzustand.)

---

## Status & TODOs

### ✅ Vollständig implementiert

- 6-DOF Knickarm URDF + SCARA URDF mit Endeffektoren ⚠️ *TCP-Offset (Saugnapf) in Yaw-Achse nicht korrekt*
- MoveIt2 Konfiguration für beide Roboter
- Gazebo Harmonic — beide Roboter in einer Simulation, namespaced Controller
- Zellenszene: Säulen, Fixiereinheit, Förderbänder, Toasterteile, 4 Schrauben
- HMI startet alles automatisch (~25 s)
- TCP-Steuerung (Translation + Rotation)
- Gelenkswinkel live mit Direktsteuerung (Slider loslassen → Roboter fährt)
- Gelenkswinkel-Anzeige in Grad und Radiant
- J3 SCARA (Linearachse) Anzeige in mm
- Pose speichern mit automatischer Joint-Konfiguration + Auto-Nummerierung
- Pose bearbeiten (Name/Aktion ohne Neu-Teachen)
- Pose duplizieren und Reihenfolge ändern (↑↓)
- Konfigurationsauswahl mit Anfahren / Konfig / Elbow Flip
- Klickbare Pose-Tabelle mit Sequenz-Highlighting
- Sequenz mit Starten / Pause / Resume / Stop / Loop
- Loop-Modus mit Scene-Reset-Aktion für saubere Wiederholung
- Multi-Object Pick (bis zu 6 Objekte gleichzeitig, radius-basiert)
- SCARA Schrauben-Sequenz (Single Pick → 20 mm runter → J4 2× drehen → absetzen → zurück)
- PICK / PLACE / SCREW / UNSCREW / Home / Scene Reset manuelle Buttons
- E-STOP mit sofortigem `mg->stop()` auf beiden Robotern
- Geschwindigkeits-Override (5–100 %) live anwendbar
- CSV Import + Export mit Joint-Werten, konfigurierbarer Pfad
- Zeitgestempeltes Log-Panel mit Farbkodierung
- Quit-Button mit sauberem Shutdown

### 🟡 Teilweise implementiert

- Pick/Place — Greifer-Simulation aktiv (visuelle Verfolgung via Gazebo set_pose), kein physischer `link_attacher`
- Screw/Unscrew — SCARA J4-Rotation animiert, aber keine Physik-Interaktion mit der Schraube

### ❌ Noch offen

- [ ] Vollautomatischer kombinierter Knickarm + SCARA Workflow
- [ ] Dokumentation als PDF
- [ ] Backup-Video der Simulation

### 💡 Nice-to-Have

- [ ] Schutzzaun in der Szene
- [ ] TCP-Offset Kalibrierung (Saugnapf ~130 mm off-center in Yaw) — wurde bis nach Demo aufgeschoben

---

## Branches

| Branch | Inhalt |
|---|---|
| `main` | Stabiler Basisstand — beide Roboter, Szene, MoveIt2 Konfiguration |
| `pick_place` | Aktiver Entwicklungsbranch — HMI, Sequenz, CSV, Multi-Object Pick & Place |