#include "path_planner.h"
#include <iostream>
#include "helpers.h"

// Local Types
// -----------------------------------------------------------------------------

// Local Constants
// -----------------------------------------------------------------------------

enum {N_PATH_POINTS = 50};

auto SAMPLE_DURATION = 0.02;

auto PREFERRED_SPEED = 20.;

auto SPEED_LIMIT = 22.35;

enum {N_LANES = 3};

auto LANE_WIDTH = 4.;

auto ROAD_WIDTH = LANE_WIDTH * N_LANES;

auto PREFERRED_BUFFER = 20.;

auto PLANNING_TIME = 3.;

auto TRAJECTORY_TIME = 1.;

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
    auto n_missing_points = N_PATH_POINTS - previous_states_s_.size();
    auto planning_time = PLANNING_TIME - TRAJECTORY_TIME
                         + static_cast<double>(n_missing_points)
                           * SAMPLE_DURATION;
    auto farthest_planned_s = current_s;
    Vehicle::State begin_s;
    Vehicle::State begin_d;
    Vehicle::State target_s = {20. * planning_time, 20, 0};
    Vehicle::State target_d = {6, 0, 0};
    if (!previous_states_s_.empty()) {
      begin_s = previous_states_s_.back();
      farthest_planned_s = begin_s[0];
      begin_s[0] = 0;
      begin_d = previous_states_d_.back();
    } else {
      begin_s = {0, 0, 0};
      begin_d = {current_d, 0, 0};
    }

    // TODO: Do the planning once a second.
    auto other_vehicles = coordinate_converter_.GetVehicles(current_s,
                                                            sensor_fusion);

    // Determine next planner state.
    if (!planner_state_) {
      planner_state_.reset(new PlannerStateKeepingLane(1));
    }
    // Dry-run the trajectory generator to determine next s and d.
    auto trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                     target_s, target_d,
                                                     planning_time,
                                                     VehicleMap(),
                                                     ROAD_WIDTH, SPEED_LIMIT);
//    std::cout << "previous_states_s_.size() " << previous_states_s_.size() << ", n_missing_points " << n_missing_points << std::endl;
    auto t = static_cast<double>(n_missing_points) * SAMPLE_DURATION;
    auto next_s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t)
                + farthest_planned_s - current_s;
    auto next_d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
    auto new_planner_state = planner_state_->GetState(N_LANES, LANE_WIDTH,
                                                      PREFERRED_BUFFER,
                                                      PREFERRED_SPEED,
                                                      TRAJECTORY_TIME,
                                                      next_s, next_d,
                                                      other_vehicles);
    if (new_planner_state) {
      auto target_vehicle_id = -1;
      std::size_t target_lane = 0;
      new_planner_state->GetTarget(target_vehicle_id, target_lane);
      std::cout << "target_vehicle_id " << target_vehicle_id
                << ", target_lane " << target_lane << std::endl;
      planner_state_ = new_planner_state;
    }

    // Run full trajecrory generation.
    trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                target_s, target_d,
                                                planning_time,
                                                other_vehicles,
                                                ROAD_WIDTH, SPEED_LIMIT);

    for (auto i = 1; i < n_missing_points + 1; ++i) {
      auto t = static_cast<double>(i) * SAMPLE_DURATION;
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
