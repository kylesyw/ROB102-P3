#include <iostream>
#include <cmath>
#include <queue>
#include <stack>
#include <unordered_set>
#include <functional>
#include <vector>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>
using namespace std;

std::vector<Cell> depthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path;
    initGraph(graph);

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    std::stack<int> visit_stack;
    visit_stack.push(start_idx);
    graph.nodes[start_idx].visited = true;

    while (!visit_stack.empty())
    {
        int current = visit_stack.top();
        visit_stack.pop();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(goal_idx, graph);
        }

        for (int neighbor : findNeighbors(current, graph))
        {
            if (!graph.nodes[neighbor].visited)
            {
                graph.nodes[neighbor].visited = true;
                graph.nodes[neighbor].parent = current;
                visit_stack.push(neighbor);
            }
        }
    }

    return {}; // Return an empty path if no path is found
}

std::vector<Cell> breadthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path;
    initGraph(graph);

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    /*if (start_idx < 0 || start_idx >= graph.nodes.size() || goal_idx < 0 || goal_idx >= graph.nodes.size()) {
        std::cerr << "Error: Start or goal index out of bounds.\n";
        return {};
    }*/

    std::queue<int> visit_queue;
    visit_queue.push(start_idx);
    graph.nodes[start_idx].visited = true;
    graph.nodes[start_idx].cost = 0;

    while (!visit_queue.empty())
    {
        int current = visit_queue.front();
        visit_queue.pop();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(goal_idx, graph);
        }

        for (int neighbor : findNeighbors(current, graph))
        {
            //std::cerr << "Checking neighbor: " << neighbor << "\n";
            /*if (neighbor < 0 || neighbor >= graph.nodes.size()) {
                std::cerr << "Warning: Skipping invalid neighbor index " << neighbor << ".\n";
                continue;
            }*/

            Cell current_cell = idxToCell(current, graph);
            Cell neighbor_cell = idxToCell(neighbor, graph);

            float distance = std::sqrt(std::pow(current_cell.i - neighbor_cell.i, 2) +
                                       std::pow(current_cell.j - neighbor_cell.j, 2));

            if (checkCollision(neighbor, graph)) {
                continue;
            }

            if (!graph.nodes[neighbor].visited || graph.nodes[current].cost + distance < graph.nodes[neighbor].cost)
            {
                graph.nodes[neighbor].visited = true;
                graph.nodes[neighbor].cost = graph.nodes[current].cost + distance;
                graph.nodes[neighbor].parent = current;
                visit_queue.push(neighbor);
            }
        }
    }
    return {};
}

bool depthLimitedSearch(GridGraph &graph, int current, int goal, int depth)
{
    if (current == goal) return true;
    if (depth <= 0) return false;

    graph.visited_cells.push_back(idxToCell(current, graph));

    for (int neighbor : findNeighbors(current, graph))
    {
        if (!graph.nodes[neighbor].visited)
        {
            graph.nodes[neighbor].visited = true;
            graph.nodes[neighbor].parent = current;
            if (depthLimitedSearch(graph, neighbor, goal, depth - 1))
            {
                return true;
            }
        }
    }
    return false;
}

std::vector<Cell> iterativeDeepeningSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    initGraph(graph);

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    int depth = 0;
    while (true)
    {
        initGraph(graph);
        if (depthLimitedSearch(graph, start_idx, goal_idx, depth))
        {
            return tracePath(goal_idx, graph);
        }
        depth++;
    }
}

float heuristic(const Cell &a, const Cell &b)
{
    return std::sqrt(std::pow(a.i - b.i, 2) + std::pow(a.j - b.j, 2));
}

std::vector<Cell> aStarSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    initGraph(graph);

    int start_idx = cellToIdx(start.i, start.j, graph);
    int goal_idx = cellToIdx(goal.i, goal.j, graph);

    auto compare = [&](int a, int b) {
        return graph.nodes[a].cost + heuristic(idxToCell(a, graph), goal) >
               graph.nodes[b].cost + heuristic(idxToCell(b, graph), goal);
    };
    std::priority_queue<int, std::vector<int>, decltype(compare)> open_set(compare);

    graph.nodes[start_idx].cost = 0;
    open_set.push(start_idx);

    while (!open_set.empty())
    {
        int current = open_set.top();
        open_set.pop();

        graph.visited_cells.push_back(idxToCell(current, graph));

        if (current == goal_idx)
        {
            return tracePath(goal_idx, graph);
        }

        for (int neighbor : findNeighbors(current, graph))
        {
            float tentative_cost = graph.nodes[current].cost + 1; 
            if (tentative_cost < graph.nodes[neighbor].cost)
            {
                graph.nodes[neighbor].cost = tentative_cost;
                graph.nodes[neighbor].parent = current;
                open_set.push(neighbor);
            }
        }
    }

    return {}; 
}
