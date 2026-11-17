// Exercises the real upstream preprocessing and IMU code in one translation unit.
// Point offsets are milliseconds, measured from the preceding estimator update.
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "preprocess.h"
#include "IMU_Processing.h"
#include "../src/preprocess.cpp"
#include "../src/IMU_Processing.cpp"

static void require(bool condition, const char *message)
{
  if (!condition) throw std::runtime_error(message);
}

static void preprocess_snapshot()
{
  sensor_msgs::PointCloud2::Ptr message(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2Modifier modifier(*message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5);
  const float values[5][3] = {{2, 3, 4}, {0.1f, 0, 0},
      {std::numeric_limits<float>::quiet_NaN(), 0, 0},
      {0, std::numeric_limits<float>::infinity(), 0}, {-2, 1, 0}};
  sensor_msgs::PointCloud2Iterator<float> x(*message, "x");
  for (const auto &point : values) {
    x[0] = point[0]; x[1] = point[1]; x[2] = point[2]; ++x;
  }
  Preprocess pre;
  pre.set(false, 8, 1.0, 1);
  pre.blind_sqr = 1.0;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  pre.process(message, output);
  require(output->size() == 2, "snapshot must reject near, NaN and infinite XYZ");
  require((*output)[0].x == 2 && (*output)[1].x == -2,
          "snapshot coordinates must retain acquisition geometry");
  for (const auto &point : output->points) {
    require(point.curvature == 0 && point.intensity == 0,
            "XYZ snapshot must not invent time offsets or intensity");
  }
  modifier.resize(0);
  pre.process(message, output);
  require(output->empty(), "empty message must not reuse the preceding cloud");
}

static sensor_msgs::Imu::Ptr imu(double time)
{
  sensor_msgs::Imu::Ptr sample(new sensor_msgs::Imu);
  sample->header.stamp.fromSec(time);
  sample->linear_acceleration.z = 9.81;
  return sample;
}

static void motion_compensation()
{
  ImuProcess processor;
  processor.set_imu_init_frame_num(1);
  StatesGroup state;
  LidarMeasureGroup group;
  PointCloudXYZI::Ptr output(new PointCloudXYZI);
  group.lio_vio_flg = LIO;
  MeasureGroup initial;
  initial.lio_time = 1.0;
  initial.imu.push_back(imu(0.99));
  initial.imu.push_back(imu(1.0));
  group.measures.push_back(initial);
  processor.Process2(group, state, output);
  require(!processor.imu_need_init, "fixture must initialize the real IMU processor");
  state.vel_end = V3D(1, 0, 0);

  // An empty image interval must advance IMU state without retaining stale poses.
  MeasureGroup empty;
  empty.lio_time = 1.1;
  empty.imu.push_back(imu(1.05));
  empty.imu.push_back(imu(1.1));
  group.measures.clear();
  group.measures.push_back(empty);
  processor.Process2(group, state, output);
  require(output->empty(), "empty interval must not expose stale points");
  require(std::abs(state.pos_end.x() - 0.1) < 1e-6,
          "initialization must set a valid IMU propagation origin");

  // Three snapshots: at interval start, middle and end. Each point is x=4
  // at its actual observation time. At 1 m/s, the ending frame sees 3.9/3.95/4.
  const float offsets[] = {0, 50, 100};
  for (float offset : offsets) {
    PointType point{};
    point.x = 4; point.curvature = offset;
    group.pcl_proc_cur->push_back(point);
  }
  MeasureGroup moving;
  moving.lio_time = 1.2;
  for (double stamp : {1.125, 1.15, 1.175, 1.2}) moving.imu.push_back(imu(stamp));
  group.measures.clear();
  group.measures.push_back(moving);
  processor.Process2(group, state, output);
  require(output->size() == 3, "all snapshots must survive compensation");
  const double expected[] = {3.9, 3.95, 4.0};
  for (size_t i = 0; i < 3; ++i)
    require(std::abs((*output)[i].x - expected[i]) < 1e-5,
            "rigid snapshot alignment must include offset zero exactly once");

  // Positive first offset catches the upstream backward loop processing its
  // first point again for each earlier IMU pose.
  group.pcl_proc_cur->clear();
  for (float offset : {50.f, 50.f}) {
    PointType point{};
    point.x = 4; point.curvature = offset;
    group.pcl_proc_cur->push_back(point);
  }
  moving.lio_time = 1.3;
  moving.imu.clear();
  for (double stamp : {1.225, 1.25, 1.275, 1.3}) moving.imu.push_back(imu(stamp));
  group.measures.clear();
  group.measures.push_back(moving);
  processor.Process2(group, state, output);
  require(output->size() == 2, "rigid snapshot point count must be preserved");
  for (const auto &point : output->points)
    require(std::abs(point.x - 3.95) < 1e-5,
            "equal-time points must undergo the same single rigid transform");
}

int main(int argc, char **argv)
{
  ros::Time::init();
  try {
    if (argc == 1 || std::string(argv[1]) == "preprocess") preprocess_snapshot();
    if (argc == 1 || std::string(argv[1]) == "imu") motion_compensation();
    std::cout << "FAST-LIVO2 snapshot regression checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
