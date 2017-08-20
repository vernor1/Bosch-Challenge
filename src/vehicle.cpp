#include "vehicle.h"
#include <cassert>

// Constants
// -----------------------------------------------------------------------------

const std::vector<double> Vehicle::kSigmaS{10.0, 4.0, 2.0};
const std::vector<double> Vehicle::kSigmaD{1.0, 1.0, 1.0};

// Public Methods
// -----------------------------------------------------------------------------

Vehicle::Vehicle(const State& begin_s, const State& begin_d)
  : begin_s_(begin_s), begin_d_(begin_d) {
  assert(begin_s_.size() == kStateOrder);
  assert(begin_d_.size() == kStateOrder);
}

void Vehicle::GetState(double t, State& s, State& d) const {
  auto t2 = t * t;
  s = {begin_s_[0] + begin_s_[1] * t + begin_s_[2] * t2 / 2.,
       begin_s_[1] + begin_s_[2] * t,
       begin_s_[2]};
  d = {begin_d_[0] + begin_d_[1] * t + begin_d_[2] * t2 / 2.,
       begin_d_[1] + begin_d_[2] * t,
       begin_d_[2]};
}
