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

auto HALF_LANE_WIDTH = LANE_WIDTH / 2.;

auto ROAD_WIDTH = LANE_WIDTH * N_LANES;

auto PREFERRED_BUFFER = 40.;

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
  : coordinate_converter_(waypoints_x, waypoints_y, waypoints_s, max_s),
    n_remaining_planned_points_() {
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

  if (!previous_states_s_.empty() && previous_path_x.empty()) {
    std::cerr << "Previous path exhausted!" << std::endl;
  }

  std::vector<double> next_x;
  std::vector<double> next_y;
  auto n_missing_points = GetMissingPoints();
  if (n_missing_points == 0) {
    // No previous points processed, repeat the control (unlikely).
    next_x.assign(previous_path_x.begin(), previous_path_x.end());
    next_y.assign(previous_path_y.begin(), previous_path_y.end());
  } else {
    // Generate new path points.
    if (n_missing_points > n_remaining_planned_points_) {
      n_remaining_planned_points_ = 0;
    } else {
      n_remaining_planned_points_ -= n_missing_points;
    }
/*
    auto planning_time = GetPlanningTime();
    auto farthest_planned_s = current_s;
    Vehicle::State begin_s;
    Vehicle::State begin_d;
    Vehicle::State target_s = {PREFERRED_SPEED * planning_time, PREFERRED_SPEED, 0};
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
*/
    auto farthest_planned_s = current_s;
    if (!previous_states_s_.empty()) {
      farthest_planned_s = previous_states_s_.back()[0];
    }

    // TODO: Do the planning once a second.
    auto other_vehicles = coordinate_converter_.GetVehicles(current_s,
                                                            sensor_fusion);

    if (n_remaining_planned_points_ == 0) {
      // Determine next planner state.
      std::cout << "Determine next planner state" << std::endl;
      n_remaining_planned_points_ = N_PATH_POINTS;
      if (!planner_state_) {
        planner_state_.reset(new PlannerStateKeepingLane(1));
      }
      // Dry-run the trajectory generator to determine next s and d.
/*
      auto trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                       target_s, target_d,
                                                       planning_time,
                                                       VehicleMap(),
                                                       ROAD_WIDTH, SPEED_LIMIT);
*/
      auto trajectory = GenerateTrajectory(current_d, other_vehicles);

      auto t = static_cast<double>(GetMissingPoints()) * SAMPLE_DURATION;
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
        std::cout << "Planner state changed"  << std::endl;
        planner_state_ = new_planner_state;
      }
    }

    // Run full trajecrory generation.
/*
    auto trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                     target_s, target_d,
                                                     planning_time,
                                                     other_vehicles,
                                                     ROAD_WIDTH, SPEED_LIMIT);
*/
    auto trajectory = GenerateTrajectory(current_d, other_vehicles);

    next_x.assign(previous_path_x.begin(), previous_path_x.end());
    next_y.assign(previous_path_y.begin(), previous_path_y.end());
    for (auto i = 1; i < GetMissingPoints() + 1; ++i) {
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

// Private Methods
// -----------------------------------------------------------------------------

std::size_t PathPlanner::GetMissingPoints() const {
  return N_PATH_POINTS - previous_states_s_.size();
}

double PathPlanner::GetPlanningTime() const {
  auto n_missing_points = N_PATH_POINTS - previous_states_s_.size();
  return PLANNING_TIME - TRAJECTORY_TIME + static_cast<double>(n_missing_points)
                                           * SAMPLE_DURATION;
}

Vehicle::Trajectory PathPlanner::GenerateTrajectory(
  double current_d,
  const VehicleMap& other_vehicles) const {
  assert(planner_state_);
  Vehicle::Trajectory trajectory;

  auto target_vehicle_id = -1;
  std::size_t target_lane = 100;
  planner_state_->GetTarget(target_vehicle_id, target_lane);
  auto d = LANE_WIDTH * target_lane + HALF_LANE_WIDTH;
  auto planning_time = GetPlanningTime();

  Vehicle::State begin_s;
  Vehicle::State begin_d;
  if (!previous_states_s_.empty()) {
    begin_s = previous_states_s_.back();
    begin_s[0] = 0;
    begin_d = previous_states_d_.back();
  } else {
    begin_s = {0, 0, 0};
    begin_d = {current_d, 0, 0};
  }

  std::cout << "Generating trajectory for planning_time " << planning_time
            << ", begin_s ("
            << begin_s[0] << "," << begin_s[1] << "," << begin_s[2] << ")"
            << ", begin_d ("
            << begin_d[0] << "," << begin_d[1] << "," << begin_d[2] << ")";
  Vehicle::State target_d = {d, 0, 0};
  if (target_vehicle_id >= 0) {
    // Target vehicle is known, follow it.
    auto target_vehicle = other_vehicles.find(target_vehicle_id);
    assert(target_vehicle != other_vehicles.end());
    Vehicle::State target_vehicle_s0;
    Vehicle::State target_vehicle_d0;
    target_vehicle->second.GetState(planning_time, target_vehicle_s0, target_vehicle_d0);
//    auto d_diff = d - target_vehicle_d0[0];
    Vehicle::State delta_s = {-PREFERRED_BUFFER, 0, 0};
//    Vehicle::State delta_d = {d_diff, 0, 0};
    std::cout << ", target_vehicle_id " << target_vehicle_id << ", delta_s ("
              << delta_s[0] << "," << delta_s[1] << "," << delta_s[2] << ")"
              << ", target_d ("
              << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")"
              << std::endl;
    trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                target_vehicle_id,
                                                delta_s, target_d,
                                                planning_time,
                                                other_vehicles,
                                                ROAD_WIDTH, SPEED_LIMIT);
  } else {
    // Target vehicle is unknown, free run at comfortable speed.
    Vehicle::State target_s = {PREFERRED_SPEED * planning_time,
                               PREFERRED_SPEED,
                               0};
    std::cout << ", target_s ("
              << target_s[0] << "," << target_s[1] << "," << target_s[2] << ")"
              << ", target_d ("
              << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")"
              << std::endl;
    trajectory = trajectory_generator_.Generate(begin_s, begin_d,
                                                target_s, target_d,
                                                planning_time,
                                                other_vehicles,
                                                ROAD_WIDTH, SPEED_LIMIT);
  }
  return trajectory;
}
