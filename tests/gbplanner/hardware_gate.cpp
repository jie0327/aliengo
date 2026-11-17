#include <cassert>
#include <cmath>
#include <iostream>
#include "HighLevelCore.h"

using aliengo_hardware::HighLevelCore;
void armed(HighLevelCore& c) {
  c.feedback(1, false, 10., 100.);
  assert(c.stand(10., 100.));
  assert(!c.ready(10.));
  c.feedback(1, false, 10.02, 100.02);
  assert(c.ready(10.02));
  assert(c.enable(true, 10.02, 100.02));
}
int main() {
  int checks = 0;
  { HighLevelCore c; assert(!c.enable(true, 1., 1.));
    assert(c.sample(1., 1.).vx == 0.); assert(c.txMode() == 0); ++checks; }
  { HighLevelCore c; armed(c); assert(c.receive(.1, 0., .2, 100.03, 100.03, 10.03, "base"));
    auto v=c.sample(10.04,100.04); assert(v.vx == .1 && v.wz == .2); assert(c.txMode()==2); ++checks; }
  { HighLevelCore c; armed(c); c.receive(1.,1.,1.,100.03,100.03,10.03,"base");
    auto v=c.sample(10.04,100.04); assert(v.vx==.12 && v.vy==.08 && v.wz==.30); ++checks; }
  { HighLevelCore c; armed(c); c.receive(.1,0.,0.,100.03,100.03,10.03,"base");
    c.feedback(2,false,10.5,100.5); assert(c.sample(10.5,100.5).vx==0.);
    assert(c.mode()=="stand"); c.feedback(1,false,10.6,100.6);
    assert(c.sample(10.6,100.6).vx==0.); ++checks; }
  { HighLevelCore c; armed(c); c.receive(.1,0.,0.,100.03,100.03,10.03,"base");
    c.feedback(2,true,10.04,100.04); assert(c.sample(10.04,100.04).vx==0.);
    assert(!c.sending()); assert(!c.enable(true,10.04,100.04)); ++checks; }
  { HighLevelCore c; armed(c); c.receive(.1,0.,0.,100.03,100.03,10.03,"base");
    assert(c.sample(10.5,100.5).vx==0.); assert(c.mode()=="stand"); ++checks; }
  { HighLevelCore c; armed(c); c.feedback(7,false,10.04,100.04);
    assert(!c.sending()); assert(!c.stand(10.04,100.04)); ++checks; }
  { HighLevelCore c; armed(c); c.receive(.1,0.,0.,100.03,100.03,10.03,"base");
    assert(c.enable(false,10.04,100.04)); assert(c.sample(10.04,100.04).vx==0.);
    assert(c.txMode()==1); ++checks; }
  std::cout << checks << "/8 hardware gate checks passed\n";
}
