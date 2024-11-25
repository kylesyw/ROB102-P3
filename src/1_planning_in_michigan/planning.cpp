#include <queue>
#include <stack>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>


#include "planning.h"


void printPath(std::vector<int>& path, Graph& g) {
   if (path.empty()) {
       std::cout << "No path found :(\n";
       return;
   }


   std::cout << "Path: ";
   for (int i = 0; i < path.size() - 1; i++) {
       std::cout << g.data[path[i]] << " -> ";
   }
   std::cout << g.data[path.back()] << "\n";
}


std::vector<int> tracePath(int n, Graph& g) {
   std::vector<int> path;
   int curr = n;
   do {
       path.push_back(curr);
       curr = getParent(curr, g);
   } while (curr != -1);


   std::reverse(path.begin(), path.end());
   return path;
}


std::vector<int> getNeighbors(int n, Graph& g) {
   return g.edges[n];
}


std::vector<float> getEdgeCosts(int n, Graph& g) {
   return g.edge_costs[n];
}


int getParent(int n, Graph& g) {
   return g.nodes[n].parent;
}


void initGraph(Graph& g) {
   g.nodes.clear();
   for (int i = 0; i < g.data.size(); i++) {
       Node n;
       n.city = g.data[i];
       n.parent = -1;
       n.visited = false;
       n.cost = HIGH;
       g.nodes.push_back(n);
   }
}


std::vector<int> bfs(int start, int goal, Graph& g) {
   initGraph(g);
//    auto cmp = [&g](int a, int b) { return g.nodes[a].cost > g.nodes[b].cost; };
//    std::priority_queue<int, std::vector<int>, decltype(cmp)> visit_queue(cmp);

    // std::vector<int> path;
   
   std::queue<int> visit_queue;


   g.nodes[start].cost = 0;
   visit_queue.push(start);


   while (!visit_queue.empty()) {
       int current = visit_queue.front();
       visit_queue.pop();


       if (current == goal) {
           return tracePath(goal, g);
       }


       for (size_t i = 0; i < getNeighbors(current, g).size(); i++) {
           int neighbor = g.edges[current][i];
           float edge_cost = g.edge_costs[current][i];
           float new_cost = g.nodes[current].cost + edge_cost;


           if (new_cost < g.nodes[neighbor].cost) {
               g.nodes[neighbor].cost = new_cost;
               g.nodes[neighbor].parent = current;
               visit_queue.push(neighbor);
           }
       }
   }


   return {};
}


std::vector<int> dfs(int start, int goal, Graph& g) {
   initGraph(g);


   std::stack<int> visit_stack;
   visit_stack.push(start);
   g.nodes[start].cost = 0;


   while (!visit_stack.empty()) {
       int current = visit_stack.top();
       visit_stack.pop();


       if (current == goal) {
           return tracePath(goal, g);
       }


       for (size_t i = 0; i < getNeighbors(current, g).size(); i++) {
           int neighbor = g.edges[current][i];
           float edge_cost = g.edge_costs[current][i];
           float new_cost = g.nodes[current].cost + edge_cost;


           if (!g.nodes[neighbor].visited || new_cost < g.nodes[neighbor].cost) {
               g.nodes[neighbor].visited = true;
               g.nodes[neighbor].cost = new_cost;
               g.nodes[neighbor].parent = current;
               visit_stack.push(neighbor);
           }
       }
   }


   return {};
}


