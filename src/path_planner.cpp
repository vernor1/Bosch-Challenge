#include "path_planner.h"
#include <iostream>
#include "helpers.h"

// Local Types
// -----------------------------------------------------------------------------

// Local Constants
// -----------------------------------------------------------------------------

enum {N_PATH_POINTS = 55};

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Public Methods
// -----------------------------------------------------------------------------

PathPlanner::PathPlanner(const std::vector<double>& waypoints_x,
                         const std::vector<double>& waypoints_y,
                         const std::vector<double>& /*waypoints_dx*/,
                         const std::vector<double>& /*waypoints_dy*/,
                         const std::vector<double>& waypoints_s,
                         double max_s)
  : coordinate_converter_(waypoints_x, waypoints_y, waypoints_s, max_s) {
  // Empty.
}

void PathPlanner::Update(double current_x,
                         double current_y,
                         double current_s,
                         double current_d,
                         double current_yaw,
                         double current_speed,
                         const std::vector<double>& previous_path_x,
                         const std::vector<double>& previous_path_y,
                         double end_path_s,
                         double end_path_d,
                         const std::vector<DetectedVehicle>& sensor_fusion,
                         ControlFunction control_function) {
  assert(previous_path_x.size() == previous_path_y.size());
  assert(previous_states_s_.size() == previous_states_d_.size());
  assert(previous_path_x.size() <= previous_states_s_.size());

  // TODO: Do the planning once a second.
  auto other_vehicles = coordinate_converter_.GetVehicles(current_s,
                                                          sensor_fusion);
  for (const auto& ov : other_vehicles) {
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    ov.second.GetState(0, vehicle_s, vehicle_d);
/*
    std::cout << "Vehicle Id " << ov.first
              << ", s " << vehicle_s[0] << ", s_dot " << vehicle_s[1] << ", s_double_dot " << vehicle_s[2]
              << ", d " << vehicle_d[0] << ", d_dot " << vehicle_d[1] << ", d_double_dot " << vehicle_d[2]
              << std::endl;
*/
  }

  // Determine next planner state.
  if (!planner_state_) {
    planner_state_.reset(new PlannerStateKeepingLane(1));
  }
  Vehicle::State begin_s;
  Vehicle::State begin_d;
  if (previous_states_s_.empty()) {
    begin_s = {0, 0, 0};
    begin_d = {current_d, 0, 0};
  } else if (previous_states_s_.size() < 4) {
    begin_s = previous_states_s_.back();
    begin_d = previous_states_d_.back();
  } else {
    begin_s = previous_states_s_[3];
    begin_d = previous_states_d_[3];
  }
  planner_state_->GetState(4, 3, {begin_s, begin_d}, other_vehicles);

  previous_states_s_.erase(
    previous_states_s_.begin(),
    previous_states_s_.begin() + previous_states_s_.size()
      - previous_path_x.size());
  previous_states_d_.erase(
    previous_states_d_.begin(),
    previous_states_d_.begin() + previous_states_d_.size()
      - previous_path_x.size());
  std::vector<double> next_x(previous_path_x.begin(), previous_path_x.end());
  std::vector<double> next_y(previous_path_y.begin(), previous_path_y.end());

  if (!previous_states_s_.empty() && previous_path_x.empty()) {
    std::cerr << "Previous path exhausted!" << std::endl;
  }

  if (previous_states_s_.size() < N_PATH_POINTS) {
    auto farthest_planned_s = current_s;
    Vehicle::State begin_s;
    Vehicle::State begin_d;
    Vehicle::State target_s = {60, 20, 0};
    Vehicle::State target_d = {2, 0, 0};
    if (!previous_states_s_.empty()) {
      begin_s = previous_states_s_.back();
      farthest_planned_s = begin_s[0];
      begin_s[0] = 0;
      begin_d = previous_states_d_.back();
    } else {
      begin_s = {0, 0, 0};
      begin_d = {current_d, 0, 0};
    }

//    auto start = std::chrono::steady_clock::now();
    auto trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                     target_s, target_d,
                                                     3, other_vehicles,
                                                     12, 22.35);
//    auto stop = std::chrono::steady_clock::now();
//    auto diff = stop - start;
//    std::cout << "Generate completed in " << std::chrono::duration<double, std::milli>(diff).count()
//              << " ms" << std::endl;

    auto dt = 0.02;
    auto n_missing_points = N_PATH_POINTS - previous_states_s_.size();
    for (auto i = 1; i < n_missing_points + 1; ++i) {
      auto t = static_cast<double>(i) * dt;
      auto s_dot_coeffs = helpers::GetDerivative(trajectory.s_coeffs);
      auto s_double_dot_coeffs = helpers::GetDerivative(s_dot_coeffs);
      auto d_dot_coeffs = helpers::GetDerivative(trajectory.d_coeffs);
      auto d_double_dot_coeffs = helpers::GetDerivative(d_dot_coeffs);
      auto s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t)
             + farthest_planned_s;
      auto s_dot = helpers::EvaluatePolynomial(s_dot_coeffs, t);
      auto s_double_dot = helpers::EvaluatePolynomial(s_double_dot_coeffs, t);
      auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
      auto d_dot = helpers::EvaluatePolynomial(d_dot_coeffs, t);
      auto d_double_dot = helpers::EvaluatePolynomial(d_double_dot_coeffs, t);
      previous_states_s_.push_back({s, s_dot, s_double_dot});
      previous_states_d_.push_back({d, d_dot, d_double_dot});
      auto cartesian = coordinate_converter_.GetCartesian({s, d});
      next_x.push_back(cartesian.x);
      next_y.push_back(cartesian.y);
    }
  }

  if (!next_x.empty()) {
    // Control the simulator.
    control_function(next_x, next_y);
  }
}
