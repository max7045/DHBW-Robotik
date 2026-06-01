# 🤖 Maze Runner (DHBW-Robotik)
**Robotik Gruppe 4 (TINF24IT1)**

Ein Simulationsprojekt mit ROS 2 Jazzy und Gazebo Harmonic. Ein Differential-Drive-Roboter mit 2D LiDAR-Sensor navigiert autonom durch zufällig generierte 3D Labyrinthe und sucht dessen Ausgang.

Das Projekt bietet neben dem Automatik-Modus auch einen Controller-Modus für manuelle Steuerung.

---

## 🌍 1. Welt generieren
Das Labyrinth kann manuell über ein eigenständiges C++ Skript erzeugt werden. Alternativ kann ROS 2 diesen Schritt auch beim Projektstart übernehmen (`-l`).

Das Skript erstellt eine zufällige Labyrinth-Welt und speichert sie als `random_maze_cpp.sdf` im Ordner `maze_runner_ws/src/maze_runner_gazebo/worlds`. Das Labyrinth hat garantiert einen Ausgang.

<br>

Manuelles Vorgehen:

```bash
cd src/maze_runner_gazebo/src

# Generator kompilieren und ausführen
g++ -O3 generate_maze.cpp -o generate_maze
./generate_maze

# (Optional) Generierte Welt in Gazebo anschauen
gz sim ../worlds/random_maze_cpp.sdf
```

## 🚀 2. Projekt kompilieren & starten
Um das komplette Projekt zu starten steht ein Shell-Skript im Hauptverzeichnis des Workspaces `maze_runner_ws` zur Verfügung:

```bash
bash -x ./run.bash [-l] [-c] [-no-textures]
```

Das Skript hat folgende (optionale) Parameter:
- `-l`: Ein (neues) Labyrinth generieren &rarr; Ansonsten wird die zuletzt generierte Welt verwendet
- `-c`: Controller-Modus aktivieren (manuelle Steuerung)
- `-no-textures`: Keine Texturen verwenden (in Gazebo) &rarr; Bessere Performance

**Achtung**: Ist beim Start kein Labyrinth vorhanden und der Parameter `-l` nicht gesetzt, wird Gazebo nicht gestartet und die Simulation ist nicht zielführend!

##

### 🎮 Controller-Modus
Im Controller-Modus kann der Roboter manuell mittels angeschlossenem Controller gesteuert, d. h. durch das Labyrinth gefahren, werden.

Der Automatik-Modus (Standard-Modus) kann jederzeit mit der Taste `A` aktiviert werden. Ab diesem Zeitpunkt übernimmt der Roboter die Kontrolle und navigiert selbstständig zum Ausgang des Labyrinths.

Controller-Steuerung:
- *Linker-Stick*: Vorwärts/Rückwärts
- *Rechter-Stick*: Links/Rechts
- *A-Taste*: Modus umschalten (Manuell / Automatik)
- *B-Taste*: Simulation zurücksetzen (Roboter zum Startpunkt teleportieren)

<br>

<u>Hinweise:</u>
- Nur mit Xbox-Controllern getestet.
- Navigation mit der Tastatur (z. B. Pfeiltasten) ist nicht implementiert.

---

## 📁 Projektstruktur
```
maze_runner_ws/
├── src/
│   ├── maze_runner_bringup/
│   │   └── launch/
│   │       └── sim.launch.xml
│   ├── maze_runner_core/
│   │   ├── CMakeLists.txt
│   │   └── src/
│   │       └── maze_runner_node.cpp
│   ├── maze_runner_description/
│   │   ├── rviz/
│   │   └── urdf/
│   └── maze_runner_gazebo/
│       ├── src/
│       │   └── generate_maze.cpp
│       ├── textures/
│       └── worlds/
│           └── random_maze_cpp.sdf
└── run.bash
```

<br>

Aufteilung der ROS-Pakete:
- `maze_runner_bringup`: ROS 2 Launch-Datei für das gesamte Projekt
- `maze_runner_core`: Implementierung der Hauptlogik zur Roboternavigation
- `maze_runner_description`: URDF-Modelle des Roboters und RViz-Konfiguration zur Visualisierung
- `maze_runner_gazebo`: Gazebo-Welt (Labyrinth) inkl. Skript zur Generierung

Eingebundene Pakete:
- `joy` &rarr; Einlesen von Gamepad-Eingaben für die manuelle Steuerung
- `slam_toolbox` &rarr; Erstellung der 2D-Karte aus den LiDAR-Daten
- `xacro` &rarr; Generierung des URDF-Robotermodells
- `rviz2` &rarr; Visualisierung von Sensordaten, Karte und Roboter
- `ros_gz_sim` &rarr; Ausführung der Gazebo-Simulation
- `ros_gz_bridge` &rarr; Austausch von Topics (Sensordaten, Steuerung) zwischen ROS 2 und Gazebo
- `robot_state_publisher` &rarr; Veröffentlichung des Roboterzustands und der Koordinatensysteme (TF)

## ⚙️ Funktionsweise
- Der Roboter wird (automatisch) am Eingang des Labyrinths platziert.
- Eine 2D-Karte wird fortlaufend anhand der LiDAR-Daten erstellt.
- Der Roboter navigiert anhand der LiDAR-Daten durch das Labyrinth. Dabei fährt er solange geradeaus, bis er auf ein Hindernis (Wand) trifft. Ein PD-Regler sorgt dabei für die Zentrierung des Roboters im Gang.
- Stößt der Roboter auf eine Wand, stoppt er und prüft die seitlichen Distanzen:
   - Ist der Weg **links** frei, dreht er sich um 90° nach links.
   - Ist links blockiert, aber **rechts** frei, dreht er sich um 90° nach rechts.
   - Sind beide Wege blockiert (Sackgasse), dreht er sich um 180° um.
- **TODO: Korrektur sobald Wegfindung implementiert**
- Sobald der LiDAR-Sensor im vorderen Halbfeld (180°) kein Hindernis mehr innerhalb eines Schwellenwerts (2 Einheiten) erkennt, stoppt er (Annahme: Ausgang gefunden).
