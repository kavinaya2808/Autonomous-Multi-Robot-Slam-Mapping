#include "mrm_exploration/frontier_detector.hpp"

#include <limits>
#include <queue>

namespace mrm_exploration
{

bool isFree(int8_t value, const DetectorParams & params)
{
  return value >= 0 && value < params.free_threshold;
}

bool isUnknown(int8_t value)
{
  return value < 0;
}

bool isOccupied(int8_t value, const DetectorParams & params)
{
  return value >= params.occupied_threshold;
}

namespace
{
// Neighbour offsets. 4-connected = up/down/left/right; 8-connected adds diagonals.
constexpr int kDx4[4] = {1, -1, 0, 0};
constexpr int kDy4[4] = {0, 0, 1, -1};
constexpr int kDx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

// A free cell is a frontier cell if any 4-neighbour is unknown. Stepping off
// the edge of the map also counts as unknown: nobody has seen beyond it.
bool isFrontierCell(const Grid & grid, int x, int y, const DetectorParams & params)
{
  if (!isFree(grid.at(x, y), params)) {
    return false;
  }
  for (int k = 0; k < 4; ++k) {
    const int nx = x + kDx4[k];
    const int ny = y + kDy4[k];
    if (!grid.inside(nx, ny) || isUnknown(grid.at(nx, ny))) {
      return true;
    }
  }
  return false;
}

long long squaredDistance(const Cell & a, const Cell & b)
{
  const long long dx = a.x - b.x;
  const long long dy = a.y - b.y;
  return dx * dx + dy * dy;
}
}  // namespace

std::vector<int> obstacleDistance(const Grid & grid, const DetectorParams & params)
{
  std::vector<int> dist(static_cast<size_t>(grid.width) * grid.height, -1);
  std::queue<Cell> queue;

  // Multi-source BFS: every obstacle is a starting point with distance 0.
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (isOccupied(grid.at(x, y), params)) {
        dist[grid.index(x, y)] = 0;
        queue.push({x, y});
      }
    }
  }

  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop();
    const int d = dist[grid.index(c.x, c.y)];
    for (int k = 0; k < 8; ++k) {
      const int nx = c.x + kDx8[k];
      const int ny = c.y + kDy8[k];
      if (grid.inside(nx, ny) && dist[grid.index(nx, ny)] < 0) {
        dist[grid.index(nx, ny)] = d + 1;
        queue.push({nx, ny});
      }
    }
  }
  return dist;
}

std::vector<Frontier> findFrontiers(const Grid & grid, const DetectorParams & params)
{
  std::vector<Frontier> frontiers;
  if (grid.width <= 0 || grid.height <= 0 || grid.data == nullptr) {
    return frontiers;
  }

  // Step 1: mark frontier cells.
  const size_t n = static_cast<size_t>(grid.width) * grid.height;
  std::vector<char> is_frontier(n, 0);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (isFrontierCell(grid, x, y, params)) {
        is_frontier[grid.index(x, y)] = 1;
      }
    }
  }

  const std::vector<int> clearance = obstacleDistance(grid, params);

  // Step 2: group neighbouring frontier cells with a flood fill (BFS).
  std::vector<char> visited(n, 0);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const int seed = grid.index(x, y);
      if (!is_frontier[seed] || visited[seed]) {
        continue;
      }

      Frontier frontier;
      std::queue<Cell> queue;
      queue.push({x, y});
      visited[seed] = 1;
      long long sum_x = 0;
      long long sum_y = 0;

      while (!queue.empty()) {
        const Cell c = queue.front();
        queue.pop();
        frontier.cells.push_back(c);
        sum_x += c.x;
        sum_y += c.y;
        for (int k = 0; k < 8; ++k) {
          const int nx = c.x + kDx8[k];
          const int ny = c.y + kDy8[k];
          if (!grid.inside(nx, ny)) {
            continue;
          }
          const int ni = grid.index(nx, ny);
          if (is_frontier[ni] && !visited[ni]) {
            visited[ni] = 1;
            queue.push({nx, ny});
          }
        }
      }

      if (static_cast<int>(frontier.cells.size()) < params.min_frontier_cells) {
        continue;  // too small to be worth a trip
      }

      const auto count = static_cast<long long>(frontier.cells.size());
      frontier.centroid = {static_cast<int>(sum_x / count), static_cast<int>(sum_y / count)};

      frontier.clearance.reserve(frontier.cells.size());
      for (const Cell & c : frontier.cells) {
        frontier.clearance.push_back(clearance[grid.index(c.x, c.y)]);
      }

      // Default target: among cells far enough from walls, take the one closest
      // to the centroid (the "middle" of the opening). If none is far enough
      // (narrow gap), take the cell with the most room around it.
      long long best_d2 = std::numeric_limits<long long>::max();
      int best_clearance = -2;
      bool found_clear = false;
      for (const Cell & c : frontier.cells) {
        const int cl = clearance[grid.index(c.x, c.y)];
        const bool clear = (cl < 0) || (cl >= params.clearance_cells);  // -1: no obstacles at all
        if (clear) {
          const long long d2 = squaredDistance(c, frontier.centroid);
          if (!found_clear || d2 < best_d2) {
            best_d2 = d2;
            frontier.target = c;
            found_clear = true;
          }
        } else if (!found_clear && cl > best_clearance) {
          best_clearance = cl;
          frontier.target = c;
        }
      }
      frontier.target_is_clear = found_clear;
      frontiers.push_back(std::move(frontier));
    }
  }
  return frontiers;
}

std::vector<int> pathDistances(const Grid & grid, Cell start, const DetectorParams & params)
{
  std::vector<int> dist(static_cast<size_t>(grid.width) * grid.height, -1);
  if (!grid.inside(start.x, start.y)) {
    return dist;
  }

  // The robot's own cell may not be "free" (e.g. map noise under the robot).
  // Search a small square around it for the nearest free cell to start from.
  if (!isFree(grid.at(start.x, start.y), params)) {
    constexpr int kSearch = 5;
    bool found = false;
    long long best = std::numeric_limits<long long>::max();
    Cell best_cell = start;
    for (int dy = -kSearch; dy <= kSearch; ++dy) {
      for (int dx = -kSearch; dx <= kSearch; ++dx) {
        const Cell c{start.x + dx, start.y + dy};
        if (grid.inside(c.x, c.y) && isFree(grid.at(c.x, c.y), params)) {
          const long long d2 = squaredDistance(c, start);
          if (d2 < best) {
            best = d2;
            best_cell = c;
            found = true;
          }
        }
      }
    }
    if (!found) {
      return dist;
    }
    start = best_cell;
  }

  // Plain BFS: every step costs 1 cell, so the first time we reach a cell is
  // along a shortest path. Only free cells can be crossed.
  std::queue<Cell> queue;
  dist[grid.index(start.x, start.y)] = 0;
  queue.push(start);
  while (!queue.empty()) {
    const Cell c = queue.front();
    queue.pop();
    const int d = dist[grid.index(c.x, c.y)];
    for (int k = 0; k < 4; ++k) {
      const int nx = c.x + kDx4[k];
      const int ny = c.y + kDy4[k];
      if (grid.inside(nx, ny) && dist[grid.index(nx, ny)] < 0 &&
        isFree(grid.at(nx, ny), params))
      {
        dist[grid.index(nx, ny)] = d + 1;
        queue.push({nx, ny});
      }
    }
  }
  return dist;
}

}  // namespace mrm_exploration
