#include "trajectory_estimator.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include "helpers.h"

using namespace std::placeholders;

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Model vehicle as circle to simplify collision detection.
const auto kVehicleRadius = 1.5;

// m/s/s
const auto kMaxAccel = 10.;

// m/s
const auto kExpectedAccelInOneSec = 1.;

// m/s/s/s
const auto kMaxJerk = 10.;

// m/s/s
const auto kExpectedJerkInOneSec = 2.;

enum {kNumberOfSamples = 100};

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Returns a value between 0 and 1 for x in the range [0, infinity] and -1 to 1
// for x in the range [-infinity, infinity].
double GetLogistic(double x) {
  return 2. / (1. + std::exp(-x)) - 1.;
}

} // namespace

const std::vector<TrajectoryEstimator::WeightedCostFunction>
    TrajectoryEstimator::kWeightedCostFunctions{
  {"TimeDiff", 10, std::bind(&TrajectoryEstimator::GetTimeDiffCost,
                             _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Sdiff", 1, std::bind(&TrajectoryEstimator::GetSdiffCost,
                         _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Ddiff", 100, std::bind(&TrajectoryEstimator::GetDdiffCost,
                          _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Collision", 1000, std::bind(&TrajectoryEstimator::GetCollisionCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Buffer", 100, std::bind(&TrajectoryEstimator::GetBufferCost,
                            _1, _2, _3, _4, _5, _6, _7, _8)},
  {"OffRoad", 1000, std::bind(&TrajectoryEstimator::GetOffRoadCost,
                              _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Speeding", 1000, std::bind(&TrajectoryEstimator::GetSpeedingCost,
                               _1, _2, _3, _4, _5, _6, _7, _8)},
  {"Efficiency", 800, std::bind(&TrajectoryEstimator::GetEfficiencyCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"MaxAccel", 1000, std::bind(&TrajectoryEstimator::GetMaxAccelCost,
                              _1, _2, _3, _4, _5, _6, _7, _8)},
  {"TotalAccel", 10, std::bind(&TrajectoryEstimator::GetTotalAccelCost,
                               _1, _2, _3, _4, _5, _6, _7, _8)},
  {"MaxJerk", 1000, std::bind(&TrajectoryEstimator::GetMaxJerkCost,
                             _1, _2, _3, _4, _5, _6, _7, _8)},
  {"TotalJerk", 10, std::bind(&TrajectoryEstimator::GetTotalJerkCost,
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
    is_s_double_dot_coeffs_computed_(),
    is_trajectory_s_computed_(),
    trajectory_s_(kNumberOfSamples),
    is_trajectory_d_computed_(),
    trajectory_d_(kNumberOfSamples),
    is_target_s_double_dot_computed_(),
    target_s_double_dot_(kNumberOfSamples),
    is_target_jerk_computed_(),
    target_jerk_(kNumberOfSamples),
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
  is_s_double_dot_coeffs_computed_ = false;
  is_trajectory_s_computed_ = false;
  is_trajectory_d_computed_ = false;
  is_target_s_double_dot_computed_ = false;
  is_target_jerk_computed_ = false;
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
  is_s_double_dot_coeffs_computed_ = false;
  is_trajectory_s_computed_ = false;
  is_trajectory_d_computed_ = false;
  is_target_s_double_dot_computed_ = false;
  is_target_jerk_computed_ = false;
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

void TrajectoryEstimator::ComputeSDoubleDotCoeffs(
  const std::vector<double>& s_coeffs) {
  if (!is_s_double_dot_coeffs_computed_) {
    ComputeSDotCoeffs(s_coeffs);
    s_double_dot_coeffs_ = helpers::GetDerivative(s_dot_coeffs_);
    is_s_double_dot_coeffs_computed_ = true;
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

void TrajectoryEstimator::ComputeTargetJerk(
  double time,
  const std::vector<double>& s_coeffs) {
  if (!is_target_jerk_computed_) {
    ComputeTargetT(time);
    ComputeSDoubleDotCoeffs(s_coeffs);
    auto jerk_coeffs = helpers::GetDerivative(s_double_dot_coeffs_);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      target_jerk_[i] = helpers::EvaluatePolynomial(jerk_coeffs, target_t_[i]);
    }
    is_target_jerk_computed_ = true;
  }
}

// Calculates the closest distance to any vehicle during a trajectory.
double TrajectoryEstimator::GetClosestDistanceToAnyVehicle(
  const Vehicle::Trajectory& trajectory,
  const VehicleMap& vehicles) {
  auto vehicle_id = -1;
  auto closest_distance = std::numeric_limits<double>::max();
//  auto start = std::chrono::steady_clock::now();

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
    } else {
//      std::cout << "Not interested in vehicle Id " << v.first << std::endl;
    }
  }

//  auto dt = trajectory.time / kNumberOfSamples;
  ComputeTrajectoryS(trajectory.time, trajectory.s_coeffs);
  ComputeTrajectoryD(trajectory.time, trajectory.d_coeffs);
  for (auto i = 0; i < kNumberOfSamples; ++i) {
//    auto t = static_cast<double>(i) * dt;
//    auto s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t);
//    auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
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

//  auto stop = std::chrono::steady_clock::now();
//  auto diff = stop - start;
//  std::cout << "GetClosestDistanceToAnyVehicle completed in "
//            << std::chrono::duration<double, std::milli>(diff).count()
//            << " ms" << std::endl;
//  if (closest_distance < 2. * kVehicleRadius) {
//    std::cout << "Colliding with vehicle Id " << vehicle_id << ", distance "
//              << closest_distance << std::endl;
//  }
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
  assert(polyAndDerivatives.size() == Vehicle::STATE_ORDER);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(helpers::EvaluatePolynomial(
      polyAndDerivatives[i], trajectory.time) - target_s[i]);
    cost += GetLogistic(diff / Vehicle::SIGMA_S[i]);
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
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.d_coeffs}};
  auto d_dot_coeffs = helpers::GetDerivative(trajectory.d_coeffs);
  polyAndDerivatives.push_back(d_dot_coeffs);
  polyAndDerivatives.push_back(helpers::GetDerivative(d_dot_coeffs));
  assert(polyAndDerivatives.size() == Vehicle::STATE_ORDER);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(helpers::EvaluatePolynomial(
      polyAndDerivatives[i], trajectory.time) - target_d[i]);
    cost += GetLogistic(diff / Vehicle::SIGMA_D[i]);
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
/*
  auto target_v = target_s[0] / trajectory.time;
  std::cout << "trajectory.time " << trajectory.time
            << ", avg_v " << avg_v
            << ", target_v " << target_v
            << ", target_s_dot " << target_s[1]
            << std::endl;
*/
  return GetLogistic(2. * (target_s[1] - avg_v) / avg_v);
}

double TrajectoryEstimator::GetMaxAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetSDoubleDot(target_time, trajectory.s_coeffs);
  std::set<double> accels;
  for (auto s_double_dot : target_s_double_dot_) {
    accels.insert(std::fabs(s_double_dot));
  }
  return *accels.rbegin() > kMaxAccel ? 1 : 0;
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
  auto total_accel = 0.;
  for (auto s_double_dot : target_s_double_dot_) {
    total_accel += std::fabs(s_double_dot * target_dt_);
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
  ComputeTargetJerk(target_time, trajectory.s_coeffs);
  std::set<double> jerks;
  for (auto jerk : target_jerk_) {
    jerks.insert(std::fabs(jerk));
  }
  return *jerks.rbegin() > kMaxJerk ? 1 : 0;
}

double TrajectoryEstimator::GetTotalJerkCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  ComputeTargetJerk(target_time, trajectory.s_coeffs);
  auto total_jerk = 0.;
  for (auto jerk : target_jerk_) {
    total_jerk += std::fabs(jerk * target_dt_);
  }
  auto jerk_per_second = total_jerk / target_time;
  return GetLogistic(jerk_per_second / kExpectedJerkInOneSec);
}
