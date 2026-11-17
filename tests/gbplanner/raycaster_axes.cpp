#include <voxblox/integrator/integrator_utils.h>
#include <cassert>
#include <iostream>
#include <set>

int main() {
  // Cardinal rays must advance on their nonzero axis. The pinned upstream
  // divides zero components by zero and repeatedly revisits the origin voxel.
  for (int axis=0;axis<3;++axis) for (int direction : {-1,1}) {
    voxblox::Point start(50,0,10),end=start;
    end[axis]+=20*direction;
    voxblox::RayCaster ray(start,end);
    voxblox::GlobalIndex index;
    std::set<int> visited;
    while(ray.nextRayIndex(&index)) {
      visited.insert(index[axis]);
      for(int fixed=0;fixed<3;++fixed) if(fixed!=axis) assert(index[fixed]==int(start[fixed]));
    }
    assert(visited.size()==21);
    assert(visited.count(int(end[axis]))==1);
  }
  std::cout << "all six cardinal ray directions reach all 21 voxels without stepping fixed axes: PASS\n";
}
