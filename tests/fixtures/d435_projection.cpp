#include <realsense_gazebo_plugin/d435_projection.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace gazebo::d435;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << std::endl;
    std::exit(1);
  }
}

int main() {
  require(std::string(ImageEncoding("R8G8B8")) == "rgb8", "Gazebo RGB alias rejected");
  require(std::string(ImageEncoding("RGB_INT8")) == "rgb8", "legacy RGB alias rejected");
  require(std::string(ImageEncoding("L8")) == "8UC1", "Gazebo infrared alias rejected");
  require(std::string(ImageEncoding("L_INT8")) == "8UC1", "legacy infrared alias rejected");
  require(ImageEncoding("unsupported") == nullptr, "unknown pixel format accepted");
  const double pi = 3.14159265358979323846;
  const auto depth = Pinhole::FromImage(848, 480, 87.0 * pi / 180.0);
  const auto color = Pinhole::FromImage(1280, 720, 69.0 * pi / 180.0);
  Point point;
  require(Unproject(depth, 424, 240, 2.0, point), "valid center depth rejected");
  require(point[0] == 0.0 && point[1] == 0.0 && point[2] == 2.0,
          "cloud principal point differs from CameraInfo");

  RigidTransform transform;
  Pixel pixel;
  require(ProjectColor(color, transform, point, pixel), "center color rejected");
  require(pixel.u == 640 && pixel.v == 360, "different color resolution ignored");

  // RGB lies 15 mm left of depth in physical coordinates, so depth center
  // appears to the right in RGB optical coordinates.
  RigidTransform physical;
  physical.translation[1] = -0.015;
  transform = OpticalTransform(physical);
  require(ProjectColor(color, transform, point, pixel), "baseline point rejected");
  require(pixel.u == 647 && pixel.v == 360, "RGB baseline sign is reversed");
  physical.translation[1] = 0.015;
  transform = OpticalTransform(physical);
  require(ProjectColor(color, transform, point, pixel), "opposite baseline rejected");
  require(pixel.u == 633, "opposite RGB baseline sign is reversed");

  // A hand-picked optical ray has u=873,v=453 with the color HFOV, not the
  // depth intrinsics. Reusing depth row/column would give another sample.
  transform = RigidTransform();
  point = {{0.5, 0.2, 2.0}};
  require(ProjectColor(color, transform, point, pixel), "off-center point rejected");
  require(pixel.u == 873 && pixel.v == 453, "RGB HFOV projection is wrong");

  // Rotate the depth ray +90 degrees about optical Z before projection.
  physical = RigidTransform();
  physical.rotation = {{1, 0, 0, 0, 0, -1, 0, 1, 0}};
  transform = OpticalTransform(physical);
  point = {{0.2, 0, 2}};
  require(ProjectColor(color, transform, point, pixel), "rotated ray rejected");
  require(pixel.u == 640 && pixel.v == 453, "extrinsic rotation ignored");

  transform = RigidTransform();
  point = {{2, 0, 2}};
  require(!ProjectColor(color, transform, point, pixel), "outside RGB FOV sampled");
  require(point[0] == 2 && point[2] == 2, "uncolored depth XYZ was changed");
  point = {{0, 0, -2}};
  require(!ProjectColor(color, transform, point, pixel), "behind-camera point sampled");
  point = {{0, 0, 0}};
  require(!ProjectColor(color, transform, point, pixel), "zero depth sampled");
  point = {{0, 0, std::numeric_limits<double>::quiet_NaN()}};
  require(!ProjectColor(color, transform, point, pixel), "NaN projection accepted");
  require(!Unproject(depth, 424, 240, 0, point), "zero depth unprojected");
  require(!Unproject(depth, 424, 240, std::numeric_limits<double>::infinity(), point),
          "infinite depth unprojected");

  require(DepthMillimeters(1.234f, 0.2f, 10.f) == 1234, "depth millimeters wrong");
  require(DepthMillimeters(0.1f, 0.2f, 10.f) == 0, "near clipping ignored");
  require(DepthMillimeters(11.f, 0.2f, 10.f) == 0, "far clipping ignored");
  require(DepthMillimeters(66.f, 0.2f, 100.f) == 0, "uint16 depth overflow");
  require(DepthMillimeters(std::numeric_limits<float>::quiet_NaN(), 0.2f, 10.f) == 0,
          "NaN depth converted to uint16");
  require(DepthMillimeters(std::numeric_limits<float>::infinity(), 0.2f, 10.f) == 0,
          "infinite depth converted to uint16");
}
