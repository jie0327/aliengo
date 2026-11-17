#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include "voxblox_ros/native_cloud_queue.h"

int main() {
  // A blocked camera TF must not withhold ready lidar observations. Identical
  // stamps from different origins and late observations must all survive.
  voxblox::NativeCloudQueue<int> queue;
  assert(queue.push("d435_depth_optical_frame", 200, 1));
  assert(queue.push("mid360_link", 200, 2));
  assert(queue.push("mid360_link", 100, 3));
  assert(!queue.push("mid360_link", 200, 4));
  int message = 0;
  assert(queue.popReady([](const std::string& frame, uint64_t, int) {
    return frame == "mid360_link";
  }, &message));
  assert(message == 2);
  assert(queue.popReady([](const std::string& frame, uint64_t, int) {
    return frame == "mid360_link";
  }, &message));
  assert(message == 3);
  assert(queue.popReady([](const std::string&, uint64_t, int) { return true; }, &message));
  assert(message == 1);

  voxblox::NativeCloudQueue<int> same_source_wait;
  same_source_wait.push("mid360_link",200,1);
  same_source_wait.push("mid360_link",100,2);
  assert(same_source_wait.popReady([](const std::string&,uint64_t stamp,int) {
    return stamp==100;
  },&message));
  assert(message==2);

  std::cout << "native queue equal/late stamps and independent TF waits: PASS\n";
}
