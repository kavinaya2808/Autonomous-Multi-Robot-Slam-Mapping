// frontier_detector: pure C++ frontier detection on an occupancy grid.
//
// No ROS in here on purpose: the algorithm takes a grid and returns
// frontiers, so it can be unit-tested with tiny hand-drawn grids
// (see test/test_frontier_detector.cpp) without starting Unity or ROS.
#pragma once

#include <cstdint>
#include <vector>

namespace mrm_exploration
{

// Read-only view of an occupancy grid, laid out exactly like
// nav_msgs/OccupancyGrid::data: row-major, -1 unknown, 0 free ... 100 occupied.
struct Grid
{
  int width{0};
  int height{0};
  const int8_t * data{nullptr};

  bool inside(int x, int y) const {return x >= 0 && y >= 0 && x < width && y < height;}
  int index(int x, int y) const {return y * width + x;}
  int8_t at(int x, int y) const {return data[index(x, y)];}
};

struct Cell
{
  int x{0};
  int y{0};
};

struct DetectorParams
{
  int free_threshold{25};      // 0 <= value < free_threshold      -> free
  int occupied_threshold{65};  // value >= occupied_threshold       -> obstacle
  int min_frontier_cells{8};   // smaller groups are ignored (lidar noise)
  int clearance_cells{6};      // preferred distance between goal and obstacles
};

struct Frontier
{
  std::vector<Cell> cells;        // every frontier cell in this group
  std::vector<int> clearance;     // per cell: distance to nearest obstacle (cells), -1 = none
  Cell centroid;                  // average position of the cells
  Cell target;                    // the cell we would send a robot to
  bool target_is_clear{false};    // target respects clearance_cells
};

bool isFree(int8_t value, const DetectorParams & params);
bool isUnknown(int8_t value);
bool isOccupied(int8_t value, const DetectorParams & params);

// Steps 1 + 2: find free cells next to unknown space, group neighbouring ones
// (8-connected flood fill) into frontiers, drop tiny groups, pick a target cell.
std::vector<Frontier> findFrontiers(const Grid & grid, const DetectorParams & params);

// Distance in cells from every cell to the nearest obstacle ("brushfire":
// a BFS that starts from all obstacles at once). 8-connected, so the value
// never overestimates the true Euclidean distance. -1 if there are no obstacles.
std::vector<int> obstacleDistance(const Grid & grid, const DetectorParams & params);

// Step 3: walking distance in cells from `start` to every cell, moving only
// through free cells (4-connected BFS). -1 = unreachable. If `start` itself is
// not free (e.g. inside an inflated blob), the nearest free cell within a few
// cells is used instead.
std::vector<int> pathDistances(const Grid & grid, Cell start, const DetectorParams & params);

}  // namespace mrm_exploration
