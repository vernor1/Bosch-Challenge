#include "trajectory_generator.h"
#include <iostream>
#include <set>
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

enum {N_SAMPLES = 10};

// Local Helper-Functions
// -----------------------------------------------------------------------------

std::vector<double> GetJmt(const Vehicle::State& begin,
                           const Vehicle::State& end,
                           double t)
{
  assert(begin.size() == Vehicle::Vehicle::STATE_ORDER);
  assert(end.size() == Vehicle::Vehicle::STATE_ORDER);
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
  assert(delta_s.size() == Vehicle::STATE_ORDER);
  Vehicle::State target_s;
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  target_vehicle.GetState(target_time, target_vehicle_s, target_vehicle_d);
  std::transform(target_vehicle_s.begin(), target_vehicle_s.end(),
                 delta_s.begin(),
                 std::back_inserter(target_s), std::plus<double>());
  assert(target_s.size() == Vehicle::STATE_ORDER);
  return target_s;
}

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

TrajectoryGenerator::TrajectoryGenerator()
  : rng_(random_device_()),
    dist_s_(0, Vehicle::SIGMA_S[0]),
    dist_s_dot_(0, Vehicle::SIGMA_S[1]),
    dist_s_double_dot_(0, Vehicle::SIGMA_S[2]),
    dist_d_(0, Vehicle::SIGMA_D[0]),
    dist_d_dot_(0, Vehicle::SIGMA_D[1]),
    dist_d_double_dot_(0, Vehicle::SIGMA_D[2]) {
}

Vehicle::Trajectory TrajectoryGenerator::Generate(const Vehicle::State& begin_s,
                                                  const Vehicle::State& begin_d,
                                                  const Vehicle::State& target_s,
                                                  const Vehicle::State& target_d,
                                                  double target_time,
                                                  const VehicleMap& vehicles,
                                                  double d_limit,
                                                  double s_dot_limit) const {
//  auto start = std::chrono::steady_clock::now();
/*
  std::cout << "Generating trajectory for begin_s ("
            << begin_s[0] << "," << begin_s[1] << "," << begin_s[2] << ")"
            << ", begin_d ("
            << begin_d[0] << "," << begin_d[1] << "," << begin_d[2] << ")"
            << ", target_s ("
            << target_s[0] << "," << target_s[1] << "," << target_s[2] << ")"
            << ", target_d ("
            << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")"
            << ", target_time " << target_time
            << ", d_limit " << d_limit
            << ", s_dot_limit " << s_dot_limit
            << std::endl;
*/
  // TODO: Replace with constant.
  auto timestep = 0.5;

  // Generate alternative goals.
  std::vector<Goal> all_goals;
  for (auto t = target_time - 4. * timestep; t <= target_time + 4. * timestep;
      t += timestep) {
    Goal base_goal{target_s, target_d, t};
    std::vector<Goal> goals;
    goals.push_back(base_goal);
    for (auto i = 0; i < N_SAMPLES; ++i) {
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

  // TODO: Remove it.
  std::set<double> all_costs;
  Vehicle::Trajectory best_trajectory;
  for (const auto& trajectory : trajectories) {
    auto cost = trajectory_estimator_.GetCost(trajectory,
                                              target_s, target_d,
                                              target_time, vehicles,
                                              d_limit, s_dot_limit);
    all_costs.insert(cost);

    if (cost < min_cost) {
      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

//  std::cout << "Other trajectory costs:";
//  for (const auto& c : all_costs) {
//    std::cout << " " << c;
//  }
//  std::cout << std::endl;

  // Print out debug data.
//  trajectory_estimator_.GetCost(best_trajectory, target_s, target_d,
//                                target_time, vehicles, d_limit, s_dot_limit,
//                                true);
/*
  auto stop = std::chrono::steady_clock::now();
  auto diff = stop - start;
  std::cout << "Generate completed in "
            << std::chrono::duration<double, std::milli>(diff).count()
            << " ms" << std::endl;
*/
  return best_trajectory;
}

Vehicle::Trajectory TrajectoryGenerator::Generate(const Vehicle::State& begin_s,
                                                  const Vehicle::State& begin_d,
                                                  std::size_t target_vehicle_id,
                                                  const Vehicle::State& delta_s,
                                                  const Vehicle::State& target_d,
                                                  double target_time,
                                                  const VehicleMap& vehicles,
                                                  double d_limit,
                                                  double s_dot_limit) const {
  auto target_vehicle = vehicles.at(target_vehicle_id);

  // Generate alternative goals.
  std::vector<Goal> all_goals;
  // TODO: Replace with constant.
  auto timestep = 0.5;
  for (auto t = target_time - 4. * timestep; t <= target_time + 4. * timestep;
      t += timestep) {
    auto target_s = GetTargetS(target_vehicle, t, delta_s);
    Goal base_goal{target_s, target_d, t};
    std::vector<Goal> goals;
    goals.push_back(base_goal);
    for (auto i = 0; i < N_SAMPLES; ++i) {
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

  // Print out debug data.
//  auto target_s = GetTargetS(target_vehicle, best_trajectory.time, delta_s);
//  CalculateCost(best_trajectory, target_s, target_d, target_time, vehicles,
//                d_limit, s_dot_limit, true);

  return best_trajectory;
}

// Private Methods
// -----------------------------------------------------------------------------

Vehicle::State TrajectoryGenerator::PerturbS(const Vehicle::State& s) const {
  assert(s.size() == Vehicle::STATE_ORDER);
  return {s[0] + dist_s_(rng_),
          s[1] + dist_s_dot_(rng_),
          s[2] + dist_s_double_dot_(rng_)};
}

Vehicle::State TrajectoryGenerator::PerturbD(const Vehicle::State& d) const {
  assert(d.size() == Vehicle::STATE_ORDER);
  return {d[0] + dist_d_(rng_),
          d[1] + dist_d_dot_(rng_),
          d[2] + dist_d_double_dot_(rng_)};
}
