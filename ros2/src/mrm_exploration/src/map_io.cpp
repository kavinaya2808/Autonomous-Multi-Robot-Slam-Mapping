#include "mrm_exploration/map_io.hpp"

#include <cstdio>
#include <fstream>
#include <iomanip>

namespace mrm_exploration
{

namespace
{
// Pixel values used by Nav2's map_saver in "trinary" mode.
constexpr unsigned char kFree = 254;
constexpr unsigned char kOccupied = 0;
constexpr unsigned char kUnknown = 205;

std::string baseName(const std::string & path)
{
  const auto slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

bool fail(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}
}  // namespace

bool saveMap(
  const Grid & grid, const MapFileInfo & info, const std::string & base_path,
  std::string * error)
{
  if (grid.width <= 0 || grid.height <= 0 || grid.data == nullptr) {
    return fail(error, "empty map");
  }

  // ---- Image (.pgm, binary "P5" greyscale) --------------------------------
  const std::string pgm_path = base_path + ".pgm";
  const std::string pgm_tmp = pgm_path + ".tmp";
  {
    std::ofstream pgm(pgm_tmp, std::ios::binary);
    if (!pgm) {
      return fail(error, "cannot write " + pgm_tmp);
    }
    pgm << "P5\n# mrm_exploration merged map\n" << grid.width << ' ' << grid.height << "\n255\n";

    const int free_limit = static_cast<int>(info.free_thresh * 100.0 + 0.5);
    const int occupied_limit = static_cast<int>(info.occupied_thresh * 100.0 + 0.5);
    // Images are stored top row first, but map row 0 is the BOTTOM of the map
    // (smallest y). So we write the rows in reverse order.
    for (int y = grid.height - 1; y >= 0; --y) {
      for (int x = 0; x < grid.width; ++x) {
        const int v = grid.at(x, y);
        unsigned char pixel = kUnknown;
        if (v >= 0 && v <= free_limit) {
          pixel = kFree;
        } else if (v >= occupied_limit) {
          pixel = kOccupied;
        }
        pgm.put(static_cast<char>(pixel));
      }
    }
    if (!pgm) {
      return fail(error, "error while writing " + pgm_tmp);
    }
  }

  // ---- Metadata (.yaml) ----------------------------------------------------
  const std::string yaml_path = base_path + ".yaml";
  const std::string yaml_tmp = yaml_path + ".tmp";
  {
    std::ofstream yaml(yaml_tmp);
    if (!yaml) {
      return fail(error, "cannot write " + yaml_tmp);
    }
    yaml << std::fixed << std::setprecision(6)
         << "image: " << baseName(pgm_path) << "\n"
         << "mode: trinary\n"
         << "resolution: " << info.resolution << "\n"
         << "origin: [" << info.origin_x << ", " << info.origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: " << info.occupied_thresh << "\n"
         << "free_thresh: " << info.free_thresh << "\n";
    if (!yaml) {
      return fail(error, "error while writing " + yaml_tmp);
    }
  }

  // Rename = atomic replace: readers see either the old or the new file.
  if (std::rename(pgm_tmp.c_str(), pgm_path.c_str()) != 0 ||
    std::rename(yaml_tmp.c_str(), yaml_path.c_str()) != 0)
  {
    return fail(error, "cannot rename map files into place");
  }
  return true;
}

long long countKnownCells(const Grid & grid)
{
  long long known = 0;
  const long long n = static_cast<long long>(grid.width) * grid.height;
  for (long long i = 0; i < n; ++i) {
    if (grid.data[i] >= 0) {
      ++known;
    }
  }
  return known;
}

}  // namespace mrm_exploration
