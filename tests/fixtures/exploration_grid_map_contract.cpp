// Validate the Python wire message with the installed official grid_map library.
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <ros/serialization.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

static void require(bool result, const char* text) {
  if (!result) throw std::runtime_error(text);
}
int main(int argc, char** argv) {
  try {
    require(argc == 2, "usage: exploration_grid_map_contract MESSAGE.bin");
    std::ifstream file(argv[1], std::ios::binary);
    require(file.good(), "message file missing");
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), {});
    ros::serialization::IStream stream(bytes.data(), bytes.size());
    grid_map_msgs::GridMap message;
    ros::serialization::deserialize(stream, message);
    grid_map::GridMap map;
    require(grid_map::GridMapRosConverter::fromMessage(message, map), "converter rejected message");
    require(map.getFrameId() == "lio_map", "frame incorrect");
    require(map.getTimestamp() == 123456789123ULL, "original nanoseconds lost");
    require(map.getSize()(0) == 2 && map.getSize()(1) == 3, "matrix dimensions transposed");
    require((map.getStartIndex() == grid_map::Index::Zero()).all(), "unexpected circular buffer");
    for (int x = 0; x < 2; ++x) for (int y = 0; y < 3; ++y) {
      grid_map::Position position;
      grid_map::Index index(x, y), recovered;
      require(map.getPosition(index, position), "index lookup failed");
      require(std::abs(position.x() - (1.25-.5*x)) < 1.e-7, "x direction reversed");
      require(std::abs(position.y() - (-1.5-.5*y)) < 1.e-7, "y direction reversed");
      require(map.getIndex(position, recovered) && (recovered == index).all(), "position lookup wrong");
      float value = map.at("elevation", index);
      if (x == 1 && y == 2) require(std::isnan(value), "unknown became ground");
      else require(std::abs(value - ((x+1)*10+y)) < 1.e-7, "column-major data wrong");
    }
    grid_map_msgs::GridMap roundtrip;
    grid_map::GridMapRosConverter::toMessage(map, roundtrip);
    require(roundtrip.data[0].layout.dim[0].label == "column_index", "outer layout wrong");
    require(roundtrip.data[0].layout.dim[1].label == "row_index", "inner layout wrong");
    std::cout << "official GridMapRosConverter: dimensions, world coordinates, values, NaN, nanosecond stamp PASS\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
