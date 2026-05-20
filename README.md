# 🏃‍♂️ Maze Runner (DHBW-Robotik)
**Robotik Gruppe 4 (TINF24IT1)**

Ein Simulationsprojekt mit ROS 2 Jazzy und Gazebo Harmonic. Ein Differential-Drive-Roboter mit LiDAR-Sensor navigiert autonom durch automatisch generierte Labyrinthe.

---

## 🌍 1. Welt generieren (C++)
Das Labyrinth wird über ein eigenständiges C++ Skript erzeugt. So kompilierst du den Generator und schaust dir die leere Welt in Gazebo an:

```bash
# In den Quellcode-Ordner wechseln
cd src/maze_runner_gazebo/src

# Generator kompilieren und ausführen
g++ -O3 generate_maze.cpp -o generate_maze
./generate_maze

# (Optional) Die generierte Welt direkt in Gazebo testen
gz sim ../worlds/random_maze_cpp.sdf
```

## 🚀 2. Projekt kompilieren & starten
Um das komplette ROS 2 Projekt (Labyrinth inklusive Roboter) zu starten, führe folgende Befehle im Hauptverzeichnis des Workspaces aus:

```bash
bash -x ./run.bash [-l] [-c] [-no-textures]
```

