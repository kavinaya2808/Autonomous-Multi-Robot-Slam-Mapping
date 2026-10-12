// Unit tests for map saving: write a tiny map, read the files back, check them.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "mrm_exploration/map_io.hpp"

using mrm_exploration::Grid;
using mrm_exploration::MapFileInfo;

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string tempBase(const std::string & name)
{
  return ::testing::TempDir() + name;
}
}  // namespace

TEST(MapIo, CountsKnownCells)
{
  const std::vector<int8_t> data = {-1, 0, 100, -1, 0, 50};
  const Grid grid{3, 2, data.data()};
  EXPECT_EQ(mrm_exploration::countKnownCells(grid), 4);
}

TEST(MapIo, WritesPgmWithTopRowFirst)
{
  // Map row 0 (bottom):  free, occupied
  // Map row 1 (top):     unknown, free
  const std::vector<int8_t> data = {0, 100, -1, 0};
  const Grid grid{2, 2, data.data()};
  const std::string base = tempBase("mrm_map_test");

  std::string error;
  ASSERT_TRUE(saveMap(grid, MapFileInfo{}, base, &error)) << error;

  const std::string pgm = readFile(base + ".pgm");
  const std::string header = "P5\n# mrm_exploration merged map\n2 2\n255\n";
  ASSERT_EQ(pgm.substr(0, header.size()), header);
  const std::string pixels = pgm.substr(header.size());
  ASSERT_EQ(pixels.size(), 4u);
  // Image is written top row first: (unknown, free), then (free, occupied).
  EXPECT_EQ(static_cast<unsigned char>(pixels[0]), 205);
  EXPECT_EQ(static_cast<unsigned char>(pixels[1]), 254);
  EXPECT_EQ(static_cast<unsigned char>(pixels[2]), 254);
  EXPECT_EQ(static_cast<unsigned char>(pixels[3]), 0);

  std::remove((base + ".pgm").c_str());
  std::remove((base + ".yaml").c_str());
}

TEST(MapIo, WritesYamlThatPointsAtTheImage)
{
  const std::vector<int8_t> data = {0};
  const Grid grid{1, 1, data.data()};
  MapFileInfo info;
  info.resolution = 0.05;
  info.origin_x = -2.5;
  info.origin_y = 1.25;
  const std::string base = tempBase("mrm_map_yaml");

  ASSERT_TRUE(saveMap(grid, info, base));
  const std::string yaml = readFile(base + ".yaml");
  EXPECT_NE(yaml.find("image: mrm_map_yaml.pgm"), std::string::npos);
  EXPECT_NE(yaml.find("resolution: 0.050000"), std::string::npos);
  EXPECT_NE(yaml.find("origin: [-2.500000, 1.250000, 0.0]"), std::string::npos);
  EXPECT_NE(yaml.find("mode: trinary"), std::string::npos);

  std::remove((base + ".pgm").c_str());
  std::remove((base + ".yaml").c_str());
}

TEST(MapIo, RejectsEmptyMap)
{
  const Grid grid{0, 0, nullptr};
  std::string error;
  EXPECT_FALSE(saveMap(grid, MapFileInfo{}, tempBase("unused"), &error));
  EXPECT_FALSE(error.empty());
}
