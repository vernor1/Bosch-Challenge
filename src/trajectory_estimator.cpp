#include "trajectory_estimator.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "helpers.h"

using namespace std::placeholders;

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Model vehicle as circle to simplify collision detection.
const auto kVehicleRadius = 1.5;

// m/s/s
const auto kMaxSAccel = 7.;

// m/s/s
const auto kMaxDAccel = 3.;

// m/s/s
const auto kMaxTotalAccel = 7.;

// m/s
const auto kExpectedAccelInOneSec = 7.;

// m/s/s/s
const auto kMaxJerk = 9.;

// m/s/s
const auto kExpectedJerkInOneSec = 9.;

// Number of samples to discretize a trajectory.
enum {kNumberOfSamples = 100};

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Returns a value between 0 and 1 for x in the range [0, infinity] and -1 to 1
// for x in the range [-infinity, infinity].
// @param[in] x  Input value.
// @return  Result of the logistic function.
double GetLogistic(double x) {
  return 2. / (1. + std::exp(-x)) - 1.;
}

} // namespace

// Cost functions and their weights for computing the integral cost.
const std::vector<TrajectoryEstimator::WeightedCostFunction>
    TrajectoryEstimator::kWeightedCostFunctions{
  {"TimeDiff", 10, std::bind(&TrajectoryEstimator::GetTimeDiffCost,
                             _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Sdiff", 1, std::bind(&TrajectoryEstimator::GetSdiffCost,
                         _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Ddiff", 10, std::bind(&TrajectoryEstimator::GetDdiffCost,
                          _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Collision", 1000, std::bind(&TrajectoryEstimator::GetCollisionCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Buffer", 100, std::bind(&TrajectoryEstimator::GetBufferCost,
                            _1, _2, _3, _4, _5, _6, _7, _8)},
  {"OffRoad", 1000, std::bind(&TrajectoryEstimator::GetOffRoadCost,
                              _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Speeding", 1000, std::bind(&TrajectoryEstimator::GetSpeedingCost,
                               _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Efficiency", 700, std::bind(&TrajectoryEstimator::GetEfficiencyCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"MaxTotalAccel", 500, std::bind(&TrajectoryEstimator::GetMaxTotalAccelCost,
                                   _1, _2, _3, _4, _5, _6, _7, _8)},
  {"TotalAccel", 500, std::bind(&TrajectoryEstimator::GetTotalAccelCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"MaxJerk", 500, std::bind(&TrajectoryEstimator::GetMaxJerkCost,
                             _1, _2, _3, _4, _5, _6, _7, _8)},
  {"TotalJerk", 500, std::bind(&TrajectoryEstimator::GetTotalJerkCost,
                               _1, _2, _3, _4, _5, _6, _7, _8)}};

// Public Methods
// -----------------------------------------------------------------------------

TrajectoryEstimator::TrajectoryEstimator()
  : is_target_dt_computed_(),
    is_trajectory_t_computed_(),
    trajectory_t_(kNumberOfSamples),
    is_target_t_computed_(),
    target_t_(kNumberOfSamples),
    is_s_dot_coeffs_computed_(),
    is_d_dot_coeffs_computed_(),
    is_s_double_dot_coeffs_computed_(),
    is_d_double_dot_coeffs_computed_(),
    is_trajectory_s_computed_(),
    trajectory_s_(kNumberOfSamples),
    is_trajectory_d_computed_(),
    trajectory_d_(kNumberOfSamples),
    is_target_s_double_dot_computed_(),
    target_s_double_dot_(kNumberOfSamples),
    is_target_d_double_dot_computed_(),
    target_d_double_dot_(kNumberOfSamples),
    is_target_s_jerk_computed_(),
    target_s_jerk_(kNumberOfSamples),
    is_target_d_jerk_computed_(),
    target_d_jerk_(kNumberOfSamples),
    is_closest_distance_computed_() {
}

double TrajectoryEstimator::GetCost(const Vehicle::Trajectory& trajectory,
                                    const Vehicle::State& target_s,
                                    const Vehicle::State& target_d,
                                    double target_time,
                                    const VehicleMap& vehicles,
                                    double d_limit,
                                    double s_dot_limit,
                                    bool is_verbose) {
  is_target_dt_computed_ = false;
  is_trajectory_t_computed_ = false;
  is_target_t_computed_ = false;
  is_s_dot_coeffs_computed_ = false;
  is_d_dot_coeffs_computed_ = false;
  is_s_double_dot_coeffs_computed_ = false;
  is_d_double_dot_coeffs_computed_ = false;
  is_trajectory_s_computed_ = false;
  is_trajectory_d_computed_ = false;
  is_target_s_double_dot_computed_ = false;
  is_target_d_double_dot_computed_ = false;
  is_target_s_jerk_computed_ = false;
  is_target_d_jerk_computed_ = false;
  is_closest_distance_computed_ = false;
  auto cost = 0.;
  if (is_verbose) {
    std::cout << "Calculating cost for trajectory.time " << trajectory.time
              << std::endl;
  }
  for (const auto& wcf : kWeightedCostFunctions) {
    auto partial_cost = wcf.weight * wcf.function(*this,
                                                  trajectory,
                                                  target_s,
                                                  target_d,
                                                  target_time,
                                                  vehicles,
                                                  d_limit,
                                                  s_dot_limit);
    if (is_verbose) {
      std::cout << "cost for " << wcf.name << " is \t " << partial_cost
      << std::endl;
    }
    cost += partial_cost;
  }
  return cost;
}

double TrajectoryEstimator::GetCost(const std::string& costFunctionName,
                                    const Vehicle::Trajectory& trajectory,
                                    const Vehicle::State& target_s,
                                    const Vehicle::State& target_d,
                                    double target_time,
                                    const VehicleMap& vehicles,
                                    double d_limit,
                                    double s_dot_limit) {
  is_target_dt_computed_ = false;
  is_trajectory_t_computed_ = false;
  is_target_t_computed_ = false;
  is_s_dot_coeffs_computed_ = false;
  is_d_dot_coeffs_computed_ = false;
  is_s_double_dot_coeffs_computed_ = false;
  is_d_double_dot_coeffs_computed_ = false;
  is_trajectory_s_computed_ = false;
  is_trajectory_d_computed_ = false;
  is_target_s_double_dot_computed_ = false;
  is_target_d_double_dot_computed_ = false;
  is_target_s_jerk_computed_ = false;
  is_target_d_jerk_computed_ = false;
  is_closest_distance_computed_ = false;
  for (const auto& wcf : kWeightedCostFunctions) {
    if (wcf.name == costFunctionName) {
      return wcf.function(*this, trajectory, target_s, target_d, target_time,
                          vehicles, d_limit, s_dot_limit);
    }
  }
  throw std::invalid_argument("Unknown cost function");
}

// Private Methods
// -----------------------------------------------------------------------------

void TrajectoryEstimator::ComputeTargetDt(double time) {
  if (!is_target_dt_computed_) {
    target_dt_ = time / kNumberOfSamples;
    is_target_dt_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTrajectoryT(double time) {
  if (!is_trajectory_t_computed_) {
    auto trajectory_dt = time / kNumberOfSamples;
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      trajectory_t_[i] = static_cast<double>(i) * trajectory_dt;
    }
    is_trajectory_t_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTargetT(double time) {
  if (!is_target_t_computed_) {
    ComputeTargetDt(time);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_t_[i] = static_cast<double>(i) * target_dt_;
    }
    is_target_t_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeSDotCoeffs(
  const std::vector<double>& s_coeffs) {
  if (!is_s_dot_coeffs_computed_) {
    s_dot_coeffs_ = helpers::GetDerivative(s_coeffs);
    is_s_dot_coeffs_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeDDotCoeffs(
  const std::vector<double>& d_coeffs) {
  if (!is_d_dot_coeffs_computed_) {
    d_dot_coeffs_ = helpers::GetDerivative(d_coeffs);
    is_d_dot_coeffs_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeSDoubleDotCoeffs(
  const std::vector<double>& s_coeffs) {
  if (!is_s_double_dot_coeffs_computed_) {
    ComputeSDotCoeffs(s_coeffs);
    s_double_dot_coeffs_ = helpers::GetDerivative(s_dot_coeffs_);
    is_s_double_dot_coeffs_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeDDoubleDotCoeffs(
  const std::vector<double>& d_coeffs) {
  if (!is_d_double_dot_coeffs_computed_) {
    ComputeDDotCoeffs(d_coeffs);
    d_double_dot_coeffs_ = helpers::GetDerivative(d_dot_coeffs_);
    is_d_double_dot_coeffs_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTrajectoryS(
  double time,
  const std::vector<double>& s_coeffs) {
  if (!is_trajectory_s_computed_) {
    ComputeTrajectoryT(time);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      trajectory_s_[i] = helpers::EvaluatePolynomial(s_coeffs,
                                                     trajectory_t_[i]);
    }
    is_trajectory_s_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTrajectoryD(
  double time,
  const std::vector<double>& d_coeffs) {
  if (!is_trajectory_d_computed_) {
    ComputeTrajectoryT(time);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      trajectory_d_[i] = helpers::EvaluatePolynomial(d_coeffs,
                                                     trajectory_t_[i]);
    }
    is_trajectory_d_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTargetSDoubleDot(
  double time,
  const std::vector<double>& s_coeffs) {
  if (!is_target_s_double_dot_computed_) {
    ComputeTargetT(time);
    ComputeSDoubleDotCoeffs(s_coeffs);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_s_double_dot_[i] = helpers::EvaluatePolynomial(
        s_double_dot_coeffs_, target_t_[i]);
    }
    is_target_s_double_dot_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTargetDDoubleDot(
  double time,
  const std::vector<double>& d_coeffs) {
  if (!is_target_d_double_dot_computed_) {
    ComputeTargetT(time);
    ComputeDDoubleDotCoeffs(d_coeffs);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_d_double_dot_[i] = helpers::EvaluatePolynomial(
        d_double_dot_coeffs_, target_t_[i]);
    }
    is_target_d_double_dot_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTargetSJerk(
  double time,
  const std::vector<double>& s_coeffs) {
  if (!is_target_s_jerk_computed_) {
    ComputeTargetT(time);
    ComputeSDoubleDotCoeffs(s_coeffs);
    auto s_jerk_coeffs = helpers::GetDerivative(s_double_dot_coeffs_);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_s_jerk_[i] = helpers::EvaluatePolynomial(s_jerk_coeffs,
                                                      target_t_[i]);
    }
    is_target_s_jerk_computed_ = true;
  }
}

void TrajectoryEstimator::ComputeTargetDJerk(
  double time,
  const std::vector<double>& d_coeffs) {
  if (!is_target_d_jerk_computed_) {
    ComputeTargetT(time);
    ComputeDDoubleDotCoeffs(d_coeffs);
    auto d_jerk_coeffs = helpers::GetDerivative(d_double_dot_coeffs_);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_d_jerk_[i] = helpers::EvaluatePolynomial(d_jerk_coeffs,
                                                      target_t_[i]);
    }
    is_target_d_jerk_computed_ = true;
  }
}

// Calculates the closest distance to any vehicle during a trajectory.
double TrajectoryEstimator::GetClosestDistanceToAnyVehicle(
  const Vehicle::Trajectory& trajectory,
  const VehicleMap& vehicles) {
  auto vehicle_id = -1;
  auto closest_distance = std::numeric_limits<double>::max();

  // Don't take vehicles following the car into consideration.
  VehicleMap vehicles_of_interest;
  auto d0 = helpers::EvaluatePolynomial(trajectory.d_coeffs, 0);
  for (const auto& v : vehicles) {
    Vehicle::State vehicle_s0;
    Vehicle::State vehicle_d0;
    v.second.GetState(0, vehicle_s0, vehicle_d0);
    if (std::fabs(d0 - vehicle_d0[0]) > kVehicleRadius
        || vehicle_s0[0] > -kVehicleRadius) {
      vehicles_of_interest.insert(v);
    }
  }

  ComputeTrajectoryS(trajectory.time, trajectory.s_coeffs);
  ComputeTrajectoryD(trajectory.time, trajectory.d_coeffs);
  for (auto i = 0; i < kNumberOfSamples; ++i) {
    for (const auto& v : vehicles_of_interest) {
      Vehicle::State vehicle_s;
      Vehicle::State vehicle_d;
      v.second.GetState(trajectory_t_[i], vehicle_s, vehicle_d);
      auto diff_s = trajectory_s_[i] - vehicle_s[0];
      auto diff_d = trajectory_d_[i] - vehicle_d[0];
      auto distance = std::sqrt(diff_s * diff_s + diff_d * diff_d);
      if (distance < closest_distance) {
        closest_distance = distance;
        vehicle_id = v.first;
      }
    }
  }

  return closest_distance;
}

double TrajectoryEstimator::GetTimeDiffCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  return GetLogistic(std::fabs(trajectory.time - target_time) / target_time);
}

double TrajectoryEstimator::GetSdiffCost(const Vehicle::Trajectory& trajectory,
                                     const Vehicle::State& target_s,
                                     const Vehicle::State& /*target_d*/,
                                     double /*target_time*/,
                                     const VehicleMap& vehicles,
                                     double /*d_limit*/,
                                     double /*s_dot_limit*/) {
  ComputeSDoubleDotCoeffs(trajectory.s_coeffs);
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.s_coeffs}};
  polyAndDerivatives.push_back(s_dot_coeffs_);
  polyAndDerivatives.push_back(s_double_dot_coeffs_);
  assert(polyAndDerivatives.size() == Vehicle::kStateOrder);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(helpers::EvaluatePolynomial(
      polyAndDerivatives[i], trajectory.time) - target_s[i]);
    cost += GetLogistic(diff / Vehicle::kSigmaS[i]);
  }

  return cost;
}

double TrajectoryEstimator::GetDdiffCost(const Vehicle::Trajectory& trajectory,
                                     const Vehicle::State& /*target_s*/,
                                     const Vehicle::State& target_d,
                                     double /*target_time*/,
                                     const VehicleMap& vehicles,
                                     double /*d_limit*/,
                                     double /*s_dot_limit*/) {
  ComputeDDoubleDotCoeffs(trajectory.d_coeffs);
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.d_coeffs}};
  polyAndDerivatives.push_back(d_dot_coeffs_);
  polyAndDerivatives.push_back(d_double_dot_coeffs_);
  assert(polyAndDerivatives.size() == Vehicle::kStateOrder);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(helpers::EvaluatePolynomial(
      polyAndDerivatives[i], trajectory.time) - target_d[i]);
    cost += GetLogistic(diff / Vehicle::kSigmaD[i]);
  }

  return cost;
}

double TrajectoryEstimator::GetCollisionCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double /*target_time*/,
  const VehicleMap& vehicles,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  if (!is_closest_distance_computed_) {
    closest_distance_ = GetClosestDistanceToAnyVehicle(trajectory, vehicles);
    is_closest_distance_computed_ = true;
  }
  return closest_distance_ < 2. * kVehicleRadius ? 1 : 0;
}

double TrajectoryEstimator::GetBufferCost(const Vehicle::Trajectory& trajectory,
                                      const Vehicle::State& /*target_s*/,
                                      const Vehicle::State& /*target_d*/,
                                      double /*target_time*/,
                                      const VehicleMap& vehicles,
                                      double /*d_limit*/,
                                      double /*s_dot_limit*/) {
  if (!is_closest_distance_computed_) {
    closest_distance_ = GetClosestDistanceToAnyVehicle(trajectory, vehicles);
    is_closest_distance_computed_ = true;
  }
  return GetLogistic(2. * kVehicleRadius / closest_distance_);
}

double TrajectoryEstimator::GetOffRoadCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/,
  double d_limit,
  double s_dot_limit) {
  ComputeTrajectoryD(trajectory.time, trajectory.d_coeffs);
  for (auto d : trajectory_d_) {
    if (d < kVehicleRadius || d > d_limit - kVehicleRadius) {
      return 1;
    }
  }
  return 0;
}

double TrajectoryEstimator::GetSpeedingCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double s_dot_limit) {
  ComputeTrajectoryT(trajectory.time);
  ComputeSDotCoeffs(trajectory.s_coeffs);
  for (auto i = 0; i < kNumberOfSamples; ++i) {
    auto s_dot = helpers::EvaluatePolynomial(s_dot_coeffs_, trajectory_t_[i]);
    if (s_dot < 0 || s_dot > s_dot_limit) {
      return 1;
    }
  }
  return 0;
}

double TrajectoryEstimator::GetEfficiencyCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& target_s,
  const Vehicle::State& /*target_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  auto avg_v = helpers::EvaluatePolynomial(trajectory.s_coeffs, trajectory.time)
               / trajectory.time;
  return GetLogistic(2. * (target_s[1] - avg_v) / avg_v);
}

double TrajectoryEstimator::GetMaxSAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSDoubleDot(target_time, trajectory.s_coeffs);
  auto max_accel = 0.;
  for (auto s_double_dot : target_s_double_dot_) {
    auto abs_accel = std::fabs(s_double_dot);
    if (abs_accel > max_accel) {
      max_accel = abs_accel;
    }
  }
  return max_accel > kMaxSAccel ? 1 : 0;
}

double TrajectoryEstimator::GetMaxDAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetDDoubleDot(target_time, trajectory.d_coeffs);
  auto max_accel = 0.;
  for (auto d_double_dot : target_d_double_dot_) {
    auto abs_accel = std::fabs(d_double_dot);
    if (abs_accel > max_accel) {
      max_accel = abs_accel;
    }
  }
  return max_accel > kMaxDAccel ? 1 : 0;
}

double TrajectoryEstimator::GetMaxTotalAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSDoubleDot(target_time, trajectory.s_coeffs);
  ComputeTargetDDoubleDot(target_time, trajectory.d_coeffs);
  assert(target_s_double_dot_.size() == target_d_double_dot_.size());
  auto max_accel = 0.;
  for (std::size_t i = 0; i < target_s_double_dot_.size(); ++i) {
    auto abs_accel = std::fabs(target_s_double_dot_[i])
                   + std::fabs(target_d_double_dot_[i]);
    if (abs_accel > max_accel) {
      max_accel = abs_accel;
    }
  }
  return max_accel > kMaxTotalAccel ? 1 : 0;
}

double TrajectoryEstimator::GetTotalAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSDoubleDot(target_time, trajectory.s_coeffs);
  ComputeTargetDDoubleDot(target_time, trajectory.d_coeffs);
  assert(target_s_double_dot_.size() == target_d_double_dot_.size());
  auto total_accel = 0.;
  for (std::size_t i = 0; i < target_s_double_dot_.size(); ++i) {
    total_accel += std::fabs(target_s_double_dot_[i] * target_dt_)
                 + std::fabs(target_d_double_dot_[i] * target_dt_);
  }
  auto accel_per_second = total_accel / target_time;
  return GetLogistic(accel_per_second / kExpectedAccelInOneSec);
}

double TrajectoryEstimator::GetMaxJerkCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSJerk(target_time, trajectory.s_coeffs);
  ComputeTargetDJerk(target_time, trajectory.d_coeffs);
  assert(target_s_jerk_.size() == target_d_jerk_.size());
  auto max_jerk = 0.;
  for (std::size_t i = 0; i < target_s_jerk_.size(); ++i) {
    auto abs_jerk = std::fabs(target_s_jerk_[i])
                  + std::fabs(target_d_jerk_[i]);
    if (abs_jerk > max_jerk) {
      max_jerk = abs_jerk;
    }
  }
  return max_jerk > kMaxJerk ? 1 : 0;
}

double TrajectoryEstimator::GetTotalJerkCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSJerk(target_time, trajectory.s_coeffs);
  ComputeTargetDJerk(target_time, trajectory.d_coeffs);
  assert(target_s_jerk_.size() == target_d_jerk_.size());
  auto total_jerk = 0.;
  for (std::size_t i = 0; i < target_s_jerk_.size(); ++i) {
    total_jerk += std::fabs(target_s_jerk_[i] * target_dt_)
                + std::fabs(target_d_jerk_[i] * target_dt_);
  }
  auto jerk_per_second = total_jerk / target_time;
  return GetLogistic(jerk_per_second / kExpectedJerkInOneSec);
}
