#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/distance_transform.h>

const float INF = std::numeric_limits<float>::infinity();

/**
 * Computes a slow distance transform by iterating through each cell multiple times.
 * This implementation uses a brute-force approach.
 */
void distanceTransformSlow(GridGraph& graph)
{
    int width = graph.width;
    int height = graph.height;

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            if (isIdxOccupied(idx, graph))
            {
                graph.obstacle_distances[idx] = 0;
            }
            else
            {
                float min_distance = INF;
                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        int obstacle_idx = cellToIdx(x, y, graph);
                        if (isIdxOccupied(obstacle_idx, graph))
                        {
                            float distance = std::sqrt(std::pow(i - x, 2) + std::pow(j - y, 2));
                            min_distance = std::min(min_distance, distance);
                        }
                    }
                }
                graph.obstacle_distances[idx] = min_distance;
            }
        }
    }
}

/**
 * Computes the Manhattan distance transform for each cell.
 * Each cell's distance is calculated as the minimum number of steps from the nearest obstacle.
 */
void distanceTransformManhattan(GridGraph& graph)
{
    int width = graph.width;
    int height = graph.height;

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            if (isIdxOccupied(idx, graph))
            {
                graph.obstacle_distances[idx] = 0;
            }
            else
            {
                float min_distance = INF;
                for (int y = 0; y < height; ++y)
                {
                    for (int x = 0; x < width; ++x)
                    {
                        int obstacle_idx = cellToIdx(x, y, graph);
                        if (isIdxOccupied(obstacle_idx, graph))
                        {
                            float distance = std::abs(i - x) + std::abs(j - y);
                            min_distance = std::min(min_distance, distance);
                        }
                    }
                }
                graph.obstacle_distances[idx] = min_distance;
            }
        }
    }
}

/**
 * One-dimensional Euclidean distance transform using a simplified approach.
 * Used for distance transforms in each row or column.
 */
std::vector<float> distanceTransformEuclidean1D(std::vector<float>& init_dt)
{
    int n = init_dt.size();
    std::vector<float> distances_out(n, INF);

    for (int i = 0; i < n; ++i)
    {
        if (init_dt[i] == 0)
        {
            distances_out[i] = 0;
        }
        else
        {
            float min_distance = INF;
            for (int j = 0; j < n; ++j)
            {
                if (init_dt[j] == 0)
                {
                    float distance = std::abs(i - j);
                    min_distance = std::min(min_distance, distance);
                }
            }
            distances_out[i] = min_distance;
        }
    }
    return distances_out;
}

/**
 * Computes a 2D Euclidean distance transform on the graph, where each cell's distance
 * is set to the Euclidean distance from the nearest obstacle.
 */
void distanceTransformEuclidean2D(GridGraph& graph)
{
    int width = graph.width;
    int height = graph.height;

    for (int j = 0; j < height; ++j)
    {
        std::vector<float> row(height, INF);
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            row[i] = isIdxOccupied(idx, graph) ? 0 : INF;
        }
        row = distanceTransformEuclidean1D(row);
        for (int i = 0; i < width; ++i)
        {
            int idx = cellToIdx(i, j, graph);
            graph.obstacle_distances[idx] = row[i];
        }
    }

    for (int i = 0; i < width; ++i)
    {
        std::vector<float> column(width, INF);
        for (int j = 0; j < height; ++j)
        {
            int idx = cellToIdx(i, j, graph);
            column[j] = graph.obstacle_distances[idx];
        }
        column = distanceTransformEuclidean1D(column);
        for (int j = 0; j < height; ++j)
        {
            int idx = cellToIdx(i, j, graph);
            graph.obstacle_distances[idx] = column[j];
        }
    }
}
