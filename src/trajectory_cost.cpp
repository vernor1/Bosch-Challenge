#include "trajectory_cost.h"
#include <cassert>
#include <iostream>
#include <set>

namespace {

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

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Returns a value between 0 and 1 for x in the range [0, infinity] and -1 to 1
// for x in the range [-infinity, infinity].
double GetLogistic(double x) {
  return 2. / (1. + std::exp(-x)) - 1.0;
}

// Evaluates a polynomial.
// @param[in] coeffs  Polynomial coefficients.
// @param[in] x       Argument to avaluate the polynomial for.
// @return            Value of the function.
double EvaluatePolynomial(const std::vector<double>& coeffs, double x) {
  auto result = 0.;
  for (std::size_t i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * std::pow(x, i);
  }
  return result;
}
/*
// Takes the coefficients of a polynomial and creates a function of time from
// them.
std::function<double(double t)> GetFunction(std::vector<double> coeffs) {
  // TODO: Replace with function evaluation.
  auto f = [coeffs](double t) {
    auto result = 0.;
    for (std::size_t i = 0; i < coeffs.size(); ++i) {
      result += coeffs.at(i) * std::pow(t, i);
    }
    return result;
  };
  return f;
}
*/
/*
// Calculates the derivative of a polynomial and returns the corresponding
// coefficients.
std::vector<double> GetDerivative(const std::vector<double>& poly_coeffs,
                                  std::size_t deriv_order = 1) {
  assert(poly_coeffs.size() >= deriv_order);
  auto result = poly_coeffs;
  for (std::size_t order = 0; order < deriv_order; ++order) {
    std::vector<double> coeffs;
    for (std::size_t i = 1; i < result.size(); ++i) {
      coeffs.push_back(static_cast<double>(i) * result[i]);
    }
    result = coeffs;
  }
  return result;
}
*/
// Calculates the derivative of a polynomial and returns the corresponding
// coefficients.
std::vector<double> GetDerivative(const std::vector<double>& coeffs) {
  assert(coeffs.size() > 0);
  std::vector<double> deriv_coeffs;
  for (std::size_t i = 1; i < coeffs.size(); ++i) {
    deriv_coeffs.push_back(static_cast<double>(i) * coeffs[i]);
  }
  return deriv_coeffs;
}
/*
std::vector<std::function<double(double t)>> GetFunctionAndDerivatives(
  std::vector<double> coeffs,
  std::size_t n) {
  std::vector<std::function<double(double t)>> functions;
  functions.push_back(GetFunction(coeffs));
  for (std::size_t i = 0; i < n; ++i) {
    coeffs = GetDerivative(coeffs);
    functions.push_back(GetFunction(coeffs));
  }
  return functions;
}
*/
double GetClosestDistance(const Vehicle::Trajectory& trajectory,
                          const Vehicle& vehicle) {
  auto closest_distance = std::numeric_limits<double>::max();
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) / N_SAMPLES * trajectory.time;
    auto s = EvaluatePolynomial(trajectory.s_coeffs, t);
    auto d = EvaluatePolynomial(trajectory.d_coeffs, t);
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    vehicle.GetState(t, vehicle_s, vehicle_d);
    auto diff_s = s - vehicle_s[0];
    auto diff_d = d - vehicle_d[0];
    auto distance = std::sqrt(diff_s * diff_s + diff_d * diff_d);
    if (distance < closest_distance) {
      closest_distance = distance;
    }
  }
  return closest_distance;
}

// Calculates the closest distance to any vehicle during a trajectory.
double GetClosestDistanceToAnyVehicle(
  const Vehicle::Trajectory& trajectory,
  const VehicleMap& vehicles) {
  auto closest_distance = std::numeric_limits<double>::max();
  for (const auto& v : vehicles) {
    auto distance = GetClosestDistance(trajectory, v.second);
    if (distance < closest_distance) {
      closest_distance = distance;
    }
  }
  return closest_distance;
}

} // namespace

// Cost Functions
// -----------------------------------------------------------------------------
double trajectory_cost::GetTimeDiffCost(const Vehicle::Trajectory& trajectory,
                                        std::size_t /*target_vehicle_id*/,
                                        const Vehicle::State& /*delta_s*/,
                                        const Vehicle::State& /*delta_d*/,
                                        double target_time,
                                        const VehicleMap& /*vehicles*/) {
  return GetLogistic(std::fabs(trajectory.time - target_time) / target_time);
}

double trajectory_cost::GetSdiffCost(const Vehicle::Trajectory& trajectory,
                                     std::size_t target_vehicle_id,
                                     const Vehicle::State& delta_s,
                                     const Vehicle::State& delta_d,
                                     double /*target_time*/,
                                     const VehicleMap& vehicles) {
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  vehicles.at(target_vehicle_id).GetState(trajectory.time,
                                          target_vehicle_s,
                                          target_vehicle_d);
  Vehicle::State target_s;
  std::transform(target_vehicle_s.begin(), target_vehicle_s.end(),
                 delta_s.begin(),
                 std::back_inserter(target_s), std::plus<double>());
  assert(target_s.size() == Vehicle::STATE_ORDER);
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.s_coeffs}};
  auto s_dot = GetDerivative(trajectory.s_coeffs);
  polyAndDerivatives.push_back(s_dot);
  polyAndDerivatives.push_back(GetDerivative(s_dot));
  assert(polyAndDerivatives.size() == Vehicle::STATE_ORDER);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(
      EvaluatePolynomial(polyAndDerivatives[i], trajectory.time) - target_s[i]);
    cost += GetLogistic(diff / Vehicle::SIGMA_S[i]);
  }

  return cost;
}

double trajectory_cost::GetDdiffCost(const Vehicle::Trajectory& trajectory,
                                     std::size_t target_vehicle_id,
                                     const Vehicle::State& delta_s,
                                     const Vehicle::State& delta_d,
                                     double /*target_time*/,
                                     const VehicleMap& vehicles) {
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  vehicles.at(target_vehicle_id).GetState(trajectory.time,
                                          target_vehicle_s,
                                          target_vehicle_d);
  Vehicle::State target_d;
  std::transform(target_vehicle_d.begin(), target_vehicle_d.end(),
                 delta_d.begin(),
                 std::back_inserter(target_d), std::plus<double>());
  assert(target_d.size() == Vehicle::STATE_ORDER);
  std::vector<std::vector<double>> polyAndDerivatives = {{trajectory.d_coeffs}};
  auto d_dot = GetDerivative(trajectory.d_coeffs);
  polyAndDerivatives.push_back(d_dot);
  polyAndDerivatives.push_back(GetDerivative(d_dot));
  assert(polyAndDerivatives.size() == Vehicle::STATE_ORDER);
  auto cost = 0.;
  for (std::size_t i = 0; i < polyAndDerivatives.size(); ++i) {
    auto diff = std::fabs(
      EvaluatePolynomial(polyAndDerivatives[i], trajectory.time) - target_d[i]);
    cost += GetLogistic(diff / Vehicle::SIGMA_D[i]);
  }

  return cost;
}

double trajectory_cost::GetCollisionCost(const Vehicle::Trajectory& trajectory,
                                         std::size_t /*target_vehicle_id*/,
                                         const Vehicle::State& /*delta_s*/,
                                         const Vehicle::State& /*delta_d*/,
                                         double /*target_time*/,
                                         const VehicleMap& vehicles) {
  auto closestDistance = GetClosestDistanceToAnyVehicle(trajectory, vehicles);
  return closestDistance < 2. * VEHICLE_RADIUS ? 1 : 0;
}

double trajectory_cost::GetBufferCost(const Vehicle::Trajectory& trajectory,
                                      std::size_t /*target_vehicle_id*/,
                                      const Vehicle::State& /*delta_s*/,
                                      const Vehicle::State& /*delta_d*/,
                                      double /*target_time*/,
                                      const VehicleMap& vehicles) {
  auto closestDistance = GetClosestDistanceToAnyVehicle(trajectory, vehicles);
  return GetLogistic(2. * VEHICLE_RADIUS / closestDistance);
}

double trajectory_cost::GetOffRoadCost(
  const Vehicle::Trajectory& /*trajectory*/,
  std::size_t /*target_vehicle_id*/,
  const Vehicle::State& /*delta_s*/,
  const Vehicle::State& /*delta_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/) {
  return 0;
}

double trajectory_cost::GetSpeedingCost(
  const Vehicle::Trajectory& /*trajectory*/,
  std::size_t /*target_vehicle_id*/,
  const Vehicle::State& /*delta_s*/,
  const Vehicle::State& /*delta_d*/,
  double /*target_time*/,
  const VehicleMap& /*vehicles*/) {
  return 0;
}

double trajectory_cost::GetEfficiencyCost(const Vehicle::Trajectory& trajectory,
                                          std::size_t target_vehicle_id,
                                          const Vehicle::State& /*delta_s*/,
                                          const Vehicle::State& /*delta_d*/,
                                          double /*target_time*/,
                                          const VehicleMap& vehicles) {
//  auto fs = GetFunction(trajectory.s_coeffs);
  auto avg_v = EvaluatePolynomial(trajectory.s_coeffs, trajectory.time)
               / trajectory.time;
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  vehicles.at(target_vehicle_id).GetState(trajectory.time,
                                          target_vehicle_s,
                                          target_vehicle_d);
  auto target_v = target_vehicle_s[0] / trajectory.time;
  return GetLogistic(2. * (target_v - avg_v) / avg_v);
}

// FIXME: Rename GetMaxAccelCost<->GetTotalAccelCost
double trajectory_cost::GetMaxAccelCost(const Vehicle::Trajectory& trajectory,
                                        std::size_t /*target_vehicle_id*/,
                                        const Vehicle::State& /*delta_s*/,
                                        const Vehicle::State& /*delta_d*/,
                                        double target_time,
                                        const VehicleMap& /*vehicles*/) {
  auto s_double_dot = GetDerivative(GetDerivative(trajectory.s_coeffs));
//  auto fa = GetFunction(s_double_dot);
  auto total_accel = 0.;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    total_accel += std::fabs(EvaluatePolynomial(s_double_dot, t) * dt);
  }
  auto accel_per_second = total_accel / target_time;
  return GetLogistic(accel_per_second / EXPECTED_ACCEL_IN_ONE_SEC);
}

double trajectory_cost::GetTotalAccelCost(const Vehicle::Trajectory& trajectory,
                                          std::size_t /*target_vehicle_id*/,
                                          const Vehicle::State& /*delta_s*/,
                                          const Vehicle::State& /*delta_d*/,
                                          double target_time,
                                          const VehicleMap& /*vehicles*/) {
  auto s_double_dot = GetDerivative(GetDerivative(trajectory.s_coeffs));
//  auto fa = GetFunction(s_double_dot);
  std::set<double> accels;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    accels.insert(std::fabs(EvaluatePolynomial(s_double_dot, t)));
  }
  return *accels.rbegin() > MAX_ACCEL ? 1 : 0;
}

double trajectory_cost::GetMaxJerkCost(const Vehicle::Trajectory& trajectory,
                                       std::size_t /*target_vehicle_id*/,
                                       const Vehicle::State& /*delta_s*/,
                                       const Vehicle::State& /*delta_d*/,
                                       double target_time,
                                       const VehicleMap& /*vehicles*/) {
  auto jerk = GetDerivative(GetDerivative(GetDerivative(trajectory.s_coeffs)));
//  auto fjerk = GetFunction(jerk);
  std::set<double> jerks;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    jerks.insert(std::fabs(EvaluatePolynomial(jerk, t)));
  }
  return *jerks.rbegin() > MAX_JERK ? 1 : 0;
}

double trajectory_cost::GetTotalJerkCost(const Vehicle::Trajectory& trajectory,
                                         std::size_t /*target_vehicle_id*/,
                                         const Vehicle::State& /*delta_s*/,
                                         const Vehicle::State& /*delta_d*/,
                                         double target_time,
                                         const VehicleMap& /*vehicles*/) {
  auto jerk = GetDerivative(GetDerivative(GetDerivative(trajectory.s_coeffs)));
//  auto fjerk = GetFunction(jerk);
  auto total_jerk = 0.;
  auto dt = target_time / N_SAMPLES;
  for (auto i = 0; i < N_SAMPLES; ++i) {
    auto t = static_cast<double>(i) * dt;
    total_jerk += std::fabs(EvaluatePolynomial(jerk, t) * dt);
  }
  auto jerk_per_second = total_jerk / target_time;
  return GetLogistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC);
}
