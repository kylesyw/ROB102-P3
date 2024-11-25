#include <stack>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "planning.h"

int main(int argc, char **argv) {
    // Load the graph from a file.
    Graph g = createGraph("../data/planning_in_michigan/mi_graph.txt");
    // Alternate graph file option:
    // Graph g = createGraph("data/planning_in_michigan/bereaf23_graph.txt");

    int start = nameToIdx("ann_arbor", g.data);
    int goal = nameToIdx("flint", g.data);
    // Alternate start and goal options:
    // int start = nameToIdx("roseville", g.data);
    // int goal = nameToIdx("saipan", g.data);
    std::vector<int> path;

    // Display neighbors of the start node.
    auto nbrs = getNeighbors(start, g);
    std::cout << "Neighbors of " << g.data[start] << ":\n";
    for (auto& n : nbrs) {
        std::cout << g.data[n] << " ";
    }
    std::cout << std::endl;

    // Display edge costs of the start node.
    auto costs = getEdgeCosts(start, g);
    std::cout << "Edge costs from " << g.data[start] << ":\n";
    for (auto& cost : costs) {
        std::cout << cost << " ";
    }
    std::cout << std::endl;

    // Perform BFS and DFS to find paths from start to goal.
    std::cout << "\nSearching for a path from " << g.data[start];
    std::cout << " (index: " << start << ") to " << g.data[goal];
    std::cout << " (index: " << goal << ")...\n";

    // BFS Pathfinding
    std::cout << "BFS:\n";
    path = bfs(start, goal, g);
    printPath(path, g);

    // DFS Pathfinding
    std::cout << "DFS:\n";
    path = dfs(start, goal, g);
    printPath(path, g);

    return 0;
}
