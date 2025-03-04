#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#define HIGH 1e6


bool isLoaded(const GridGraph& graph) {
    bool correct_size = graph.cell_odds.size() == graph.width * graph.height;
    bool positive_size = graph.width > 0 && graph.height > 0;
    bool positive_m_per_cell = graph.meters_per_cell > 0;
    return correct_size && positive_size && positive_m_per_cell;
}

bool loadFromFile(const std::string& file_path, GridGraph& graph) {
    std::ifstream in(file_path);
    if (!in.is_open()) {
        std::cerr << "ERROR: loadFromFile: Failed to load from " << file_path << std::endl;
        return false;
    }

    // Read header
    in >> graph.origin_x >> graph.origin_y >> graph.width >> graph.height >> graph.meters_per_cell;

    if (graph.width < 0 || graph.height < 0 || graph.meters_per_cell <= 0.0f) {
        return false;
    }

    graph.collision_radius = ROBOT_RADIUS + graph.meters_per_cell;

    int num_cells = graph.width * graph.height;
    graph.cell_odds.resize(num_cells);
    graph.obstacle_distances = std::vector<float>(num_cells, 0);

    int odds;
    for (int idx = 0; idx < num_cells; ++idx) {
        in >> odds;
        graph.cell_odds[idx] = odds;
    }

    initGraph(graph);
    return true;
}

/*void initGraph(GridGraph& graph) {

    for (auto& node : graph.nodes) {
        node.visited = false;
        node.parent = -1;
        node.cost = HIGH; 
    }
}*/

void initGraph(GridGraph& graph) {
    if (graph.nodes.empty()) {
        graph.nodes.resize(graph.width * graph.height);
        for (int idx = 0; idx < graph.width * graph.height; ++idx) {
            graph.nodes[idx].visited = false;
            graph.nodes[idx].parent = -1;
            graph.nodes[idx].cost = HIGH; 
        }
    } else {
        for (auto& node : graph.nodes) {
            node.visited = false;
            node.parent = -1;
            node.cost = HIGH;
        }
    }
}

std::string mapAsString(GridGraph& graph) {
    std::ostringstream oss;
    oss << graph.origin_x << " " << graph.origin_y << " ";
    oss << graph.width << " " << graph.height << " " << graph.meters_per_cell << " ";

    for (int j = 0; j < graph.height; j++) {
        for (int i = 0; i < graph.width; i++) {
            oss << +graph.cell_odds[cellToIdx(i, j, graph)] << " ";
        }
    }
    return oss.str();
}

int cellToIdx(int i, int j, const GridGraph& graph) {
    return i + j * graph.width;
}

Cell idxToCell(int idx, const GridGraph& graph) {
    Cell c;
    c.i = idx % graph.width;
    c.j = idx / graph.width;
    return c;
}

Cell posToCell(float x, float y, const GridGraph& graph) {
    int i = static_cast<int>(floor((x - graph.origin_x) / graph.meters_per_cell));
    int j = static_cast<int>(floor((y - graph.origin_y) / graph.meters_per_cell));
    Cell c = {i, j};
    return c;
}

std::vector<float> cellToPos(int i, int j, const GridGraph& graph) {
    float x = (i + 0.5) * graph.meters_per_cell + graph.origin_x;
    float y = (j + 0.5) * graph.meters_per_cell + graph.origin_y;
    return {x, y};
}

bool isCellInBounds(int i, int j, const GridGraph& graph) {
    return i >= 0 && j >= 0 && i < graph.width && j < graph.height;
}

bool isIdxOccupied(int idx, const GridGraph& graph) {
    return graph.cell_odds[idx] >= graph.threshold;
}

bool isCellOccupied(int i, int j, const GridGraph& graph) {
    return isIdxOccupied(cellToIdx(i, j, graph), graph);
}

std::vector<int> findNeighbors(int idx, const GridGraph& graph) {
    int i = idx % graph.width;
    int j = idx / graph.width;
    std::vector<int> neighbors;

    const int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1},
                                  {0, -1},           {0, 1},
                                  {1, -1}, {1, 0}, {1, 1}};

    for (const auto& dir : directions) {
        int ni = i + dir[0];
        int nj = j + dir[1];
        if (isCellInBounds(ni, nj, graph)) {
            neighbors.push_back(cellToIdx(ni, nj, graph));
        }
    }
    return neighbors;
}

bool checkCollisionFast(int idx, const GridGraph& graph) {
    return graph.obstacle_distances[idx] * graph.meters_per_cell <= graph.collision_radius;
}

bool checkCollision(int idx, const GridGraph& graph) {
    if (isIdxOccupied(idx, graph)) {
        return true;
    }

    double dtheta = graph.meters_per_cell / graph.collision_radius;
    double theta = 0;
    auto c = idxToCell(idx, graph);
    auto state = cellToPos(c.i, c.j, graph);

    while (theta < 2 * PI) {
        double x = state[0] + graph.collision_radius * cos(theta);
        double y = state[1] + graph.collision_radius * sin(theta);
        Cell c = posToCell(x, y, graph);

        if (!isCellInBounds(c.i, c.j, graph) || isCellOccupied(c.i, c.j, graph)) {
            return true;
        }
        theta += dtheta;
    }
    return false;
}

int getParent(int idx, const GridGraph& graph) {
    return graph.nodes[idx].parent;
}

float getScore(int idx, const GridGraph& graph) {
    return graph.nodes[idx].cost;
}

int findLowestScore(const std::vector<int>& node_list, const GridGraph& graph) {
    int min_idx = 0;
    for (int i = 1; i < node_list.size(); ++i) {
        if (getScore(node_list[i], graph) < getScore(node_list[min_idx], graph)) {
            min_idx = i;
        }
    }
    return min_idx;
}

std::vector<Cell> tracePath(int goal, const GridGraph& graph) {
    std::vector<Cell> path;
    int current = goal;
    while (current != -1) {
        path.push_back(idxToCell(current, graph));
        current = getParent(current, graph);
    }
    std::reverse(path.begin(), path.end());
    return path;
}