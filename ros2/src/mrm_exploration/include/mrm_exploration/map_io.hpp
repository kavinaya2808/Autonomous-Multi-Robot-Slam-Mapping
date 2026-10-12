// map_io: save an occupancy grid in the standard ROS map format, and measure it.
//
// Pure C++ (no ROS), like frontier_detector, so it can be unit-tested.
// The output is exactly what Nav2's map_server / map_saver use:
//   <name>.pgm   greyscale image: white = free, black = occupied, grey = unknown
//   <name>.yaml  resolution, origin and thresholds, pointing at the .pgm
#pragma once

#include <string>

#include "mrm_exploration/frontier_detector.hpp"

namespace mrm_exploration
{

struct MapFileInfo
{
  double resolution{0.05};   // metres per cell
  double origin_x{0.0};      // world position of cell (0, 0)'s corner
  double origin_y{0.0};
  double free_thresh{0.25};  // cell value <= 25  -> free (white)
  double occupied_thresh{0.65};  // cell value >= 65 -> occupied (black)
};

// Writes <base_path>.pgm and <base_path>.yaml. Each file is written to a
// temporary name first and then renamed, so a reader never sees half a file.
// Returns false (and fills *error) on failure.
bool saveMap(
  const Grid & grid, const MapFileInfo & info, const std::string & base_path,
  std::string * error = nullptr);

// Number of cells that are known (free or occupied, i.e. not -1).
long long countKnownCells(const Grid & grid);

}  // namespace mrm_exploration
