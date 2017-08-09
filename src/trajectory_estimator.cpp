#include "trajectory_estimator.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include "helpers.h"

using namespace std::placeholders;

namespace {

// Local Types
// -----------------------------------------------------------------------------

typedef std::function<double(TrajectoryEstimator&,
                             const Vehicle::Trajectory& trajectory,
                             const Vehicle::State& target_s,
                             const Vehicle::State& target_d,
                             double target_time,
                             const VehicleMap& vehicles,
                             double d_limit,
                             double s_dot_limit)> Function;

struct WeightedCostFunction {
  const char* name;
  double weight;
  Function function;
};

// Local Constants
// -----------------------------------------------------------------------------

// Model vehicle as circle to simplify collision detection.
const auto VEHICLE_RADIUS = 1.5;

// m/s/s
const auto MAX_ACCEL = 10.;

// m/s
const auto EXPECTED_ACCEL_IN_ONE_SEC = 1.;

// m/s/s/s
const auto MAX_JERK = 10.;

// m/s/s
const auto EXPECTED_JERK_IN_ONE_SEC = 2.;

enum {N_SAMPLES = 100};

const std::vector<WeightedCostFunction> WEIGHTED_COST_FUNCTIONS{
  {"GetTimeDiffCost", 10, std::bind(&TrajectoryEstimator::GetTimeDiffCost,
                                    _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetSdiffCost", 1, std::bind(&TrajectoryEstimator::GetSdiffCost,
                                _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetDdiffCost", 10, std::bind(&TrajectoryEstimator::GetDdiffCost,
                                 _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetEfficiencyCost", 100, std::bind(&TrajectoryEstimator::GetEfficiencyCost,
                                       _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetMaxJerkCost", 100, std::bind(&TrajectoryEstimator::GetMaxJerkCost,
                                    _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetTotalJerkCost", 10, std::bind(&TrajectoryEstimator::GetTotalJerkCost,
                                     _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetCollisionCost", 1000, std::bind(&TrajectoryEstimator::GetCollisionCost,
                                       _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetBufferCost", 200, std::bind(&TrajectoryEstimator::GetBufferCost,
                                   _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetMaxAccelCost", 100, std::bind(&TrajectoryEstimator::GetMaxAccelCost,
                                     _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetTotalAccelCost", 10, std::bind(&TrajectoryEstimator::GetTotalAccelCost,
                                      _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetOffRoadCost", 1000, std::bind(&TrajectoryEstimator::GetOffRoadCost,
                                     _1, _2, _3, _4, _5, _6, _7, _8)},
  {"GetSpeedingCost", 1000, std::bind(&TrajectoryEstimator::GetSpeedingCost,
                                      _1, _2, _3, _4, _5, _6, _7, _8)}};

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Returns a value between 0 and 1 for x in the range [0, infinity] and -1 to 1
// for x in the range [-infinity, infinity].
double GetLogistic(double x) {
  return 2. / (1. + std::exp(-x)) - 1.0;
}

// Calculates the closest distance to any vehicle during a trajectory.
double GetClosestDistanceToAnyVehicle(
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
    if (std::fabs(d0 - vehicle_d0[0]) > VEHICLE_RADIUS
        || vehicle_s0[0] > -VEHICLE_RADIUS) {
      vehicles_of_interest.insert(v);
    } else {
//      std::cout << "Not interested in vehicle Id " << v.first << std::endl;
    }
  }

  auto dt = trajectory.time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    auto s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t);
    auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
    for (const auto& v : vehicles_of_interest) {
      Vehicle::State vehicle_s;
      Vehicle::State vehicle_d;
      v.second.GetState(t, vehicle_s, vehicle_d);
      auto diff_s = s - vehicle_s[0];
      auto diff_d = d - vehicle_d[0];
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
//  if (closest_distance < 2. * VEHICLE_RADIUS) {
//    std::cout << "Colliding with vehicle Id " << vehicle_id << ", distance "
//              << closest_distance << std::endl;
//  }
  return closest_distance;
}

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

TrajectoryEstimator::TrajectoryEstimator()
  : is_closest_distance_computed_() {
}

double TrajectoryEstimator::GetCost(const Vehicle::Trajectory& trajectory,
                                    const Vehicle::State& target_s,
                                    const Vehicle::State& target_d,
                                    double target_time,
                                    const VehicleMap& vehicles,
                                    double d_limit,
                                    double s_dot_limit,
                                    bool is_verbose) {
  auto cost = 0.;
  if (is_verbose) {
    std::cout << "Calculating cost for trajectory.time " << trajectory.time
              << std::endl;
  }
  for (const auto& wcf : WEIGHTED_COST_FUNCTIONS) {
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
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.s_coeffs}};
  auto s_dot = helpers::GetDerivative(trajectory.s_coeffs);
  polyAndDerivatives.push_back(s_dot);
  polyAndDerivatives.push_back(helpers::GetDerivative(s_dot));
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
  auto d_dot = helpers::GetDerivative(trajectory.d_coeffs);
  polyAndDerivatives.push_back(d_dot);
  polyAndDerivatives.push_back(helpers::GetDerivative(d_dot));
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
  return closest_distance_ < 2. * VEHICLE_RADIUS ? 1 : 0;
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
  return GetLogistic(2. * VEHICLE_RADIUS / closest_distance_);
}

double TrajectoryEstimator::GetOffRoadCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/,
  double d_limit,
  double s_dot_limit) {
  auto dt = trajectory.time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
    if (d < VEHICLE_RADIUS || d > d_limit - VEHICLE_RADIUS) {
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
  auto s_dot_coeffs = helpers::GetDerivative(trajectory.s_coeffs);
  auto dt = trajectory.time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    auto s_dot = helpers::EvaluatePolynomial(s_dot_coeffs, t);
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
  auto s_double_dot = helpers::GetDerivative(helpers::GetDerivative(
    trajectory.s_coeffs));
  std::set<double> accels;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    accels.insert(std::fabs(helpers::EvaluatePolynomial(s_double_dot, t)));
  }
  return *accels.rbegin() > MAX_ACCEL ? 1 : 0;
}

double TrajectoryEstimator::GetTotalAccelCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  auto s_double_dot = helpers::GetDerivative(helpers::GetDerivative(
    trajectory.s_coeffs));
  auto total_accel = 0.;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    total_accel += std::fabs(helpers::EvaluatePolynomial(s_double_dot, t) * dt);
  }
  auto accel_per_second = total_accel / target_time;
  return GetLogistic(accel_per_second / EXPECTED_ACCEL_IN_ONE_SEC);
}

double TrajectoryEstimator::GetMaxJerkCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  auto jerk = helpers::GetDerivative(helpers::GetDerivative(
    helpers::GetDerivative(trajectory.s_coeffs)));
  std::set<double> jerks;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    jerks.insert(std::fabs(helpers::EvaluatePolynomial(jerk, t)));
  }
  return *jerks.rbegin() > MAX_JERK ? 1 : 0;
}

double TrajectoryEstimator::GetTotalJerkCost(
  const Vehicle::Trajectory& trajectory,
  const Vehicle::State& /*target_s*/,
  const Vehicle::State& /*target_d*/,
  double target_time,
  const VehicleMap& /*vehicles*/,
  double /*d_limit*/,
  double /*s_dot_limit*/) {
  auto jerk = helpers::GetDerivative(helpers::GetDerivative(
    helpers::GetDerivative(trajectory.s_coeffs)));
  auto total_jerk = 0.;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    total_jerk += std::fabs(helpers::EvaluatePolynomial(jerk, t) * dt);
  }
  auto jerk_per_second = total_jerk / target_time;
  return GetLogistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}
