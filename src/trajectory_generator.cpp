#include "trajectory_generator.h"
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

namespace {

// Local Types
// -----------------------------------------------------------------------------

struct Goal {
  Vehicle::State s;
  Vehicle::State d;
  double time;
};

// Local Constants
// -----------------------------------------------------------------------------

// Number of perturbed goal samples.
enum {kNumberOfSamples = 10};

// Timestep of target times.
const auto kTargetTimestep = 0.5;

// Span of the target time.
const auto kTargetTimespan = 4. * kTargetTimestep;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Computes a jerk minimizing trajectory (JMT).
// @param[in] begin  Beginning state vector.
// @param[in] end    Ending state vector.
// @param[in] t      Time.
// @return  6th order polynomial coefficients of the JMT trajectory.
std::vector<double> GetJmt(const Vehicle::State& begin,
                           const Vehicle::State& end,
                           double t)
{
  assert(begin.size() == Vehicle::Vehicle::kStateOrder);
  assert(end.size() == Vehicle::Vehicle::kStateOrder);
  auto t2 = t * t;
  auto t3 = t2 * t;
  auto t4 = t3 * t;
  auto t5 = t4 * t;

  // Matrix A:
  // t^3   t^4    t^5
  // 3t^2  4t^3   5t^4
  // 6t    12t^2  20t^3
  Eigen::MatrixXd a((Eigen::MatrixXd(3, 3)
    << t3, t4, t5,
       3 * t2, 4 * t3, 5 * t4,
       6 * t, 12 * t2, 20 * t3).finished());

  // Matrix B:
  // x<e> - (x<b> + x'<b> + 1/2*x''<b>t^2)
  // x'<e> - (x'<b> + x''<b>t)
  // x''<e> - x''<b>
  Eigen::VectorXd b(Eigen::VectorXd::Zero(3));
  b(0) = end[0] - begin[0] - begin[1] * t - begin[2] * t2 / 2.;
  b(1) = end[1] - begin[1] - begin[2] * t;
  b(2) = end[2] - begin[2];

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(a);
  Eigen::VectorXd x = dec.solve(b);

  return {begin[0], begin[1], begin[2] / 2., x(0), x(1), x(2)};
}

std::vector<Vehicle::Trajectory> GetGoalTrajectories(
  const Vehicle::State& begin_s,
  const Vehicle::State& begin_d,
  const std::vector<Goal>& goals) {
  std::vector<Vehicle::Trajectory> trajectories;
  for (const auto& goal : goals) {
    Vehicle::Trajectory trajectory;
    trajectory.time = goal.time;
    trajectory.s_coeffs = GetJmt(begin_s, goal.s, goal.time);
    trajectory.d_coeffs = GetJmt(begin_d, goal.d, goal.time);
    trajectories.push_back(trajectory);
  }
  return trajectories;
}

Vehicle::State GetTargetS(const Vehicle& target_vehicle,
                          double target_time,
                          const Vehicle::State& delta_s) {
  assert(delta_s.size() == Vehicle::kStateOrder);
  Vehicle::State target_s;
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  target_vehicle.GetState(target_time, target_vehicle_s, target_vehicle_d);
  std::transform(target_vehicle_s.begin(), target_vehicle_s.end(),
                 delta_s.begin(),
                 std::back_inserter(target_s), std::plus<double>());
  assert(target_s.size() == Vehicle::kStateOrder);
  return target_s;
}

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

TrajectoryGenerator::TrajectoryGenerator()
  : rng_(random_device_()),
    dist_s_(0, Vehicle::kSigmaS[0]),
    dist_s_dot_(0, Vehicle::kSigmaS[1]),
    dist_s_double_dot_(0, Vehicle::kSigmaS[2]),
    dist_d_(0, Vehicle::kSigmaD[0]),
    dist_d_dot_(0, Vehicle::kSigmaD[1]),
    dist_d_double_dot_(0, Vehicle::kSigmaD[2]) {
  // Empty.
}

Vehicle::Trajectory TrajectoryGenerator::Generate(
  const Vehicle::State& begin_s,
  const Vehicle::State& begin_d,
  const Vehicle::State& target_s,
  const Vehicle::State& target_d,
  double target_time,
  const VehicleMap& vehicles,
  double d_limit,
  double s_dot_limit) const {
  // Generate alternative goals.
  std::vector<Goal> all_goals;
  for (auto t = target_time - kTargetTimespan;
       t <= target_time + kTargetTimespan; t += kTargetTimestep) {
    Goal base_goal{target_s, target_d, t};
    std::vector<Goal> goals;
    goals.push_back(base_goal);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      Goal perturbed_goal;
      perturbed_goal.time = t;
      perturbed_goal.s = PerturbS(base_goal.s);
      perturbed_goal.d = PerturbD(base_goal.d);
      goals.push_back(perturbed_goal);
    }
    all_goals.insert(all_goals.end(), goals.begin(), goals.end());
  }

  // Find best trajectory.
  auto trajectories = GetGoalTrajectories(begin_s, begin_d, all_goals);
  auto min_cost = std::numeric_limits<double>::max();
  Vehicle::Trajectory best_trajectory;
  for (const auto& trajectory : trajectories) {
    auto cost = trajectory_estimator_.GetCost(trajectory,
                                              target_s, target_d,
                                              target_time, vehicles,
                                              d_limit, s_dot_limit);
    if (cost < min_cost) {
      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

  // Print out debug data.
  //trajectory_estimator_.GetCost(best_trajectory, target_s, target_d,
  //                              target_time, vehicles, d_limit, s_dot_limit,
  //                              true);

  return best_trajectory;
}

Vehicle::Trajectory TrajectoryGenerator::Generate(
  const Vehicle::State& begin_s,
  const Vehicle::State& begin_d,
  std::size_t target_vehicle_id,
  const Vehicle::State& delta_s,
  const Vehicle::State& target_d,
  double target_time,
  const VehicleMap& vehicles,
  double d_limit,
  double s_dot_limit) const {
  // Find target vehicle.
  auto target_vehicle = vehicles.at(target_vehicle_id);

  // Generate alternative goals.
  std::vector<Goal> all_goals;
  for (auto t = target_time - kTargetTimespan;
       t <= target_time + kTargetTimespan; t += kTargetTimestep) {
    auto target_s = GetTargetS(target_vehicle, t, delta_s);
    Goal base_goal{target_s, target_d, t};
    std::vector<Goal> goals;
    goals.push_back(base_goal);
    for (auto i = 0; i < kNumberOfSamples; ++i) {
      Goal perturbed_goal;
      perturbed_goal.time = t;
      perturbed_goal.s = PerturbS(base_goal.s);
      perturbed_goal.d = PerturbD(base_goal.d);
      goals.push_back(perturbed_goal);
    }
    all_goals.insert(all_goals.end(), goals.begin(), goals.end());
  }

  // Find best trajectory.
  auto trajectories = GetGoalTrajectories(begin_s, begin_d, all_goals);
  auto min_cost = std::numeric_limits<double>::max();
  Vehicle::Trajectory best_trajectory;
  for (const auto& trajectory : trajectories) {
    auto target_s = GetTargetS(target_vehicle, trajectory.time, delta_s);
    auto cost = trajectory_estimator_.GetCost(trajectory,
                                              target_s, target_d,
                                              target_time, vehicles,
                                              d_limit, s_dot_limit);
    if (cost < min_cost) {
      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

  return best_trajectory;
}

// Private Methods
// -----------------------------------------------------------------------------

Vehicle::State TrajectoryGenerator::PerturbS(const Vehicle::State& s) const {
  assert(s.size() == Vehicle::kStateOrder);
  return {s[0] + dist_s_(rng_),
          s[1] + dist_s_dot_(rng_),
          s[2] + dist_s_double_dot_(rng_)};
}

Vehicle::State TrajectoryGenerator::PerturbD(const Vehicle::State& d) const {
  assert(d.size() == Vehicle::kStateOrder);
  return {d[0] + dist_d_(rng_),
          d[1] + dist_d_dot_(rng_),
          d[2] + dist_d_double_dot_(rng_)};
}
