#!/bin/bash

USE_CTRL="false"
USE_TEXTURES="true"
NEW_LABYRINTH="false"

for arg in "$@"; do
    if [ "$arg" == "-c" ] || [ "$arg" == "-controller" ]; then
        USE_CTRL="true"
        echo "Controller-Modus aktiviert."
    fi
    if [ "$arg" == "-no-textures" ]; then
        USE_TEXTURES="false"
        echo "Texturen deaktiviert (Performance-Modus)."
    fi

    if [ "$arg" == "-l" ]; then
        NEW_LABYRINTH="true"
        echo "Neues Labyrinth generieren."
    fi
done

echo "Baue das Projekt..."
colcon build
source install/setup.bash


if [ "$NEW_LABYRINTH" == "true" ]; then
    WORLD_PATH="$(pwd)/src/maze_runner_gazebo/worlds/random_maze_cpp.sdf"
    TEXTURE_PATH="$(pwd)/src/maze_runner_gazebo/textures/simple_box.png"

    echo "Generiere Labyrinth..."
    if [ "$USE_TEXTURES" == "true" ]; then
        ros2 run maze_runner_gazebo generate_maze $WORLD_PATH $TEXTURE_PATH
    else
        ros2 run maze_runner_gazebo generate_maze $WORLD_PATH 
    fi

    colcon build
    source install/setup.bash
fi

echo "Starte Simulation..."
ros2 launch maze_runner_bringup sim.launch.xml use_controller:=$USE_CTRL