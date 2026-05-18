#!/bin/bash

# Standardmäßig ist der Controller aus
USE_CTRL="false"

# Prüfe, ob ein Parameter mitgegeben wurde (Argument 1)
if [ "$1" == "-c" ] || [ "$1" == "-controller" ]; then
    USE_CTRL="true"
    echo "Controller-Modus aktiviert."
fi

# Baue den Workspace neu (hilfreich bei C++ Änderungen)
colcon build
source install/setup.bash

# Starte die Launch-Datei und übergebe die ROS 2 Variable
ros2 launch maze_runner_bringup sim.launch.xml use_controller:=$USE_CTRL