#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <random>

using namespace std;

// Rekursive Funktion zum Fräsen der Gänge (Recursive Backtracker)
void carve_passages(int cx, int cy, vector<vector<int>>& maze, int width, int height, mt19937& gen) {
    // Definiere die vier Himmelsrichtungen (2 Schritte, um Wände stehen zu lassen)
    vector<pair<int, int>> directions = {{0, 2}, {2, 0}, {0, -2}, {-2, 0}};
    
    // Mische die Richtungen zufällig für ein einzigartiges Labyrinth
    shuffle(directions.begin(), directions.end(), gen);

    for (auto dir : directions) {
        int nx = cx + dir.first;
        int ny = cy + dir.second;

        // Prüfe, ob die Zielzelle im gültigen Bereich liegt und noch eine Wand (1) ist
        if (nx >= 1 && nx < width - 1 && ny >= 1 && ny < height - 1 && maze[ny][nx] == 1) {
            // Reisse die Wand zwischen aktueller Zelle und Zielzelle ein
            maze[cy + dir.second / 2][cx + dir.first / 2] = 0;
            maze[ny][nx] = 0; // Markiere Zielzelle als freien Weg (0)
            
            // Rekursiver Aufruf von der neuen Zelle aus
            carve_passages(nx, ny, maze, width, height, gen);
        }
    }
}

// Generiert das 2D-Raster des Labyrinths
vector<vector<int>> generate_maze_grid(int width, int height) {
    // Initialisiere das Raster komplett mit Wänden (1)
    vector<vector<int>> maze(height, vector<int>(width, 1));
    
    // Initialisiere den Zufallsgenerator (Mersenne Twister)
    random_device rd;
    mt19937 gen(rd());

    // Lege Startpunkt fest und beginne mit dem Fräsen
    maze[1][1] = 0;
    carve_passages(1, 1, maze, width, height, gen);

    // Erstelle manuell einen Eingang (oben) und Ausgang (unten)
    maze[0][1] = 0;
    maze[height - 1][width - 2] = 0;

    return maze;
}

// Wandelt das 2D-Raster in eine Gazebo SDF-Datei um
void export_to_sdf(const vector<vector<int>>& grid, const string& filename, const string& texture_path) {
    double wall_size = 1.5;
    ofstream sdf_file(filename);

    if (!sdf_file.is_open()) {
        cerr << "Fehler: Konnte Datei nicht oeffnen: " << filename << endl;
        return;
    }

    // Schreibe den XML-Header und die grundlegende Gazebo-Welt-Konfiguration
    sdf_file << "<?xml version=\"1.0\"?>\n<sdf version=\"1.9\">\n  <world name=\"maze_world\">\n";
    sdf_file << "    <physics name=\"1ms\" type=\"ignored\">\n      <max_step_size>0.001</max_step_size>\n      <real_time_factor>1.0</real_time_factor>\n    </physics>\n";
    sdf_file << "    <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\"/>\n";
    sdf_file << "    <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\"/>\n";
    sdf_file << "    <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\"/>\n\n";
    sdf_file << "    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri></include>\n";
    sdf_file << "    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri></include>\n";

    // Iteriere durch das Raster und generiere für jede '1' eine 3D-Box
    for (size_t y = 0; y < grid.size(); ++y) {
        for (size_t x = 0; x < grid[y].size(); ++x) {
            if (grid[y][x] == 1) {
                double pos_x = x * wall_size;
                double pos_y = y * wall_size;
                
                sdf_file << "    <model name='wall_" << x << "_" << y << "'>\n";
                sdf_file << "      <static>true</static>\n";
                sdf_file << "      <pose>" << pos_x << " " << pos_y << " " << wall_size/2 << " 0 0 0</pose>\n";
                sdf_file << "      <link name='link'>\n";
                sdf_file << "        <collision name='collision'><geometry><box><size>" << wall_size << " " << wall_size << " " << wall_size << "</size></box></geometry></collision>\n";
                sdf_file << "        <visual name='visual'><geometry><box><size>" << wall_size << " " << wall_size << " " << wall_size << "</size></box></geometry>\n";
                if (!texture_path.empty()) 
                {
                    sdf_file << "          <material>\n";
                    sdf_file << "            <ambient>0.8 0.8 0.8 1</ambient>\n"; // Helligkeit
                    sdf_file << "            <diffuse>0.8 0.8 0.8 1</diffuse>\n";
                    sdf_file << "            <pbr>\n";
                    sdf_file << "              <metal>\n";
                    sdf_file << "                <albedo_map>file://" << texture_path << "</albedo_map>\n";
                    sdf_file << "                <roughness>0.9</roughness>\n"; // Ziegelsteine spiegeln kaum
                    sdf_file << "              </metal>\n";
                    sdf_file << "            </pbr>\n";
                    sdf_file << "          </material>\n";
                }else {
                    sdf_file << "          <material><ambient>0.3 0.3 0.3 1</ambient></material>\n";
                }
                sdf_file << "        </visual>\n      </link>\n    </model>\n";
            }
        }
    }

    sdf_file << "  </world>\n</sdf>\n";
    sdf_file.close();
    cout << "Welt erfolgreich generiert: " << filename << endl;
}

int main(int argc, char* argv[]) {
    string output_path = "../worlds/random_maze_cpp.sdf";
    string texture_path = "";

    // Argumente auslesen (Neu: Das dritte Argument ist der Bildpfad)
    if (argc >= 2) output_path = argv[1];
    if (argc >= 3) texture_path = argv[2];
    // Generiere Labyrinth-Matrix (17x17)
    vector<vector<int>> maze_grid = generate_maze_grid(17, 17);
    
    // Speichere das Ergebnis direkt im worlds-Ordner
    export_to_sdf(maze_grid, output_path, texture_path);

    return 0;
}