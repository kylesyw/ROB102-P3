#include <iostream>
#include <fstream>
#include <string>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>
#include <path_planning/utils/viz_utils.h>
#include <path_planning/graph_search/graph_search.h>
#include <path_planning/graph_search/distance_transform.h>

/**
 * @brief Print Usage prints the command line usage for the program
 */
void print_usage()
{
    std::cout << "Usage:\n";
    std::cout << "./planner [map_file] [planning_algo] [start_x] [start_y] [goal_x] [goal_y]" << std::endl;
}

int main(int argv, char **argc)
{
    std::string map_file, planning_algo;
    Cell start, goal;
    if (argv >= 7)
    {
        // If arguments were provided on the command line, load those.
        map_file = std::string(argc[1]);
        planning_algo = std::string(argc[2]);
        start = {std::atoi(argc[3]), std::atoi(argc[4])};
        goal = {std::atoi(argc[5]), std::atoi(argc[6])};
    }
    else
    {
        // Ask the user for inputs if no arguments were provided.
        std::cout << "Path to map file: ";
        std::cin >> map_file;
        std::cout << "Start cell:\n";
        std::cout << "\ti: ";
        std::cin >> start.i;
        std::cout << "\tj: ";
        std::cin >> start.j;
        std::cout << "Goal cell:\n";
        std::cout << "\ti: ";
        std::cin >> goal.i;
        std::cout << "\tj: ";
        std::cin >> goal.j;
        std::cout << "Which algorithm would you like to use? [dfs, bfs, astar] : ";
        std::cin >> planning_algo;
    }

    // Load the graph and ensure it is loaded successfully.
    GridGraph graph;
    if (!loadFromFile(map_file, graph))
    {
        std::cerr << "Invalid map file: " << map_file << std::endl;
        return 1;
    }

    // Optional: Perform the distance transform to use checkCollisionFast.
    // distanceTransformEuclidean2D(graph);

    // Plan a path using the selected algorithm.
    std::vector<Cell> path;
    if (planning_algo == "astar")
    {
        path = aStarSearch(graph, start, goal);
    }
    else if (planning_algo == "bfs")
    {
        std::cout << "it got to bfs" << std::endl;
        path = breadthFirstSearch(graph, start, goal);
        std::cout << "it finished bfs" << std::endl;
    }
    else if (planning_algo == "dfs")
    {
        path = depthFirstSearch(graph, start, goal);
    }
    else
    {
        std::cerr << "Invalid planning algorithm: " << planning_algo << std::endl;
        return 1;
    }

    // Output the result
    if (!path.empty())
    {
        std::cout << "Found path of length: " << path.size() << "\n";
    }
    else
    {
        std::cout << "No path found.\n";
    }

    // Generate the output file for visualization.
    generatePlanFile(start, goal, path, graph, planning_algo);

    return 0;
}
