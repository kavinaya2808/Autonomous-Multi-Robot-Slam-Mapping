// Unit tests for the frontier detector, using small hand-drawn maps:
//   '#' = occupied (100)   '.' = free (0)   '?' = unknown (-1)
// Row 0 of the drawing is y = 0.
//
// Run with:  colcon test --packages-select mrm_exploration
//            colcon test-result --verbose
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "mrm_exploration/frontier_detector.hpp"

using mrm_exploration::Cell;
using mrm_exploration::DetectorParams;
using mrm_exploration::Grid;

namespace
{
// Keeps the cell data alive and hands out a Grid view of it.
struct TestMap
{
  explicit TestMap(const std::vector<std::string> & rows)
  {
    height = static_cast<int>(rows.size());
    width = static_cast<int>(rows[0].size());
    for (const auto & row : rows) {
      for (char ch : row) {
        data.push_back(ch == '#' ? 100 : (ch == '.' ? 0 : -1));
      }
    }
  }
  Grid grid() const {return Grid{width, height, data.data()};}

  int width{0};
  int height{0};
  std::vector<int8_t> data;
};

DetectorParams smallParams()
{
  DetectorParams p;
  p.min_frontier_cells = 3;
  p.clearance_cells = 1;
  return p;
}
}  // namespace

TEST(FrontierDetector, FullyMappedRoomHasNoFrontiers)
{
  TestMap map({
    "#######",
    "#.....#",
    "#.....#",
    "#.....#",
    "#######",
  });
  EXPECT_TRUE(findFrontiers(map.grid(), smallParams()).empty());
}

TEST(FrontierDetector, OpenSideOfRoomIsOneFrontier)
{
  // The right side is unexplored: the column of free cells next to it
  // (x = 3, y = 1..3) is the frontier.
  TestMap map({
    "####???",
    "#...???",
    "#...???",
    "#...???",
    "####???",
  });
  const auto frontiers = findFrontiers(map.grid(), smallParams());
  ASSERT_EQ(frontiers.size(), 1u);
  EXPECT_EQ(frontiers[0].cells.size(), 3u);
  for (const Cell & c : frontiers[0].cells) {
    EXPECT_EQ(c.x, 3);
  }
  // The target is the middle of the opening, away from the top/bottom walls.
  EXPECT_EQ(frontiers[0].target.x, 3);
  EXPECT_EQ(frontiers[0].target.y, 2);
  EXPECT_TRUE(frontiers[0].target_is_clear);
  ASSERT_EQ(frontiers[0].clearance.size(), frontiers[0].cells.size());
  EXPECT_EQ(frontiers[0].clearance[1], 2);  // middle cell: 2 cells from top/bottom walls
}

TEST(FrontierDetector, TwoSeparateOpeningsAreTwoFrontiers)
{
  TestMap map({
    "?????????",
    "#.......#",
    "#.......#",
    "####.####",
    "#.......#",
    "#.......#",
    "?????????",
  });
  // Top opening (y = 1) and bottom opening (y = 5) are not touching.
  EXPECT_EQ(findFrontiers(map.grid(), smallParams()).size(), 2u);
}

TEST(FrontierDetector, TinyFrontiersAreIgnored)
{
  TestMap map({
    "#####",
    "#...#",
    "#...?",
    "#...#",
    "#####",
  });
  auto params = smallParams();
  params.min_frontier_cells = 2;  // the gap is only 1 cell wide
  EXPECT_TRUE(findFrontiers(map.grid(), params).empty());
  params.min_frontier_cells = 1;
  EXPECT_EQ(findFrontiers(map.grid(), params).size(), 1u);
}

TEST(FrontierDetector, MapEdgeCountsAsUnknown)
{
  // Free cells on the border of the map touch "nothing seen yet".
  TestMap map({
    ".....",
    ".....",
    ".....",
  });
  const auto frontiers = findFrontiers(map.grid(), smallParams());
  ASSERT_EQ(frontiers.size(), 1u);         // the whole ring is connected
  EXPECT_EQ(frontiers[0].cells.size(), 12u);  // 15 cells minus the 3 inner ones
}

TEST(PathDistances, GoesAroundWalls)
{
  // A wall with a gap at the bottom: from (1,1) to (5,1) you must walk around.
  TestMap map({
    "#######",
    "#..#..#",
    "#..#..#",
    "#.....#",
    "#######",
  });
  const Grid g = map.grid();
  const auto dist = pathDistances(g, {1, 1}, smallParams());
  EXPECT_EQ(dist[g.index(1, 1)], 0);
  EXPECT_EQ(dist[g.index(2, 1)], 1);
  EXPECT_EQ(dist[g.index(5, 1)], 8);   // down 2, right 4, up 2
  EXPECT_EQ(dist[g.index(3, 1)], -1);  // inside the wall
}

TEST(PathDistances, UnreachableRegionIsMinusOne)
{
  TestMap map({
    "#######",
    "#..#..#",
    "#..#..#",
    "#######",
  });
  const Grid g = map.grid();
  const auto dist = pathDistances(g, {1, 1}, smallParams());
  EXPECT_EQ(dist[g.index(5, 2)], -1);
}

TEST(PathDistances, StartsFromNearestFreeCell)
{
  TestMap map({
    "#####",
    "#...#",
    "#...#",
    "#####",
  });
  const Grid g = map.grid();
  // Start inside the wall: should snap to the free cell (1,1) next to it.
  const auto dist = pathDistances(g, {0, 1}, smallParams());
  EXPECT_EQ(dist[g.index(1, 1)], 0);
  EXPECT_EQ(dist[g.index(3, 2)], 3);
}

TEST(ObstacleDistance, CountsCellsToNearestWall)
{
  TestMap map({
    "#....",
    ".....",
    ".....",
  });
  const Grid g = map.grid();
  const auto dist = obstacleDistance(g, smallParams());
  EXPECT_EQ(dist[g.index(0, 0)], 0);
  EXPECT_EQ(dist[g.index(1, 1)], 1);  // diagonal neighbour
  EXPECT_EQ(dist[g.index(4, 2)], 4);
}
