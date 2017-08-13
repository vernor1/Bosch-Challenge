#include "path_planner.h"
#include <iostream>
#include "helpers.h"

// Local Constants
// -----------------------------------------------------------------------------

enum {kNumberOfPathPoints = 50};

auto kSampleDuration = 0.02;

auto kPreferredSpeed = 20.;

auto kSpeedLimit = 22.35;

enum {kNumberOfLanes = 3};

auto kLaneWidth = 4.;

auto kHalfLaneWidth = kLaneWidth / 2.;

auto kRoadWidth = kLaneWidth * kNumberOfLanes;

auto kPlanningTime = 2.;

auto kTrajectoryTime = 1.;

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

void PathPlanner::Update(double /*current_x*/,
                         double /*current_y*/,
                         double current_s,
                         double current_d,
                         double /*current_yaw*/,
                         double /*current_speed*/,
                         const std::vector<double>& previous_path_x,
                         const std::vector<double>& previous_path_y,
                         double /*end_path_s*/,
                         double /*end_path_d*/,
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

    Vehicle::State nearest_s = !previous_states_s_.empty()
                               ? previous_states_s_.front()
                               : Vehicle::State{current_s, 0, 0};
    Vehicle::State nearest_d = !previous_states_d_.empty()
                               ? previous_states_d_.front()
                               : Vehicle::State{current_d, 0, 0};

    auto other_vehicles = coordinate_converter_.GetVehicles(nearest_s[0],
                                                            sensor_fusion);

    if (n_remaining_planned_points_ == 0) {
      // Determine next planner state.
      std::cout << "Determine next planner state" << std::endl;
      n_remaining_planned_points_ = kNumberOfPathPoints;
      if (!planner_state_) {
        planner_state_.reset(new PlannerStateKeepingLane(1));
      }
      // Dry-run the trajectory generator to determine next s and d.
      auto trajectory = GenerateTrajectory(current_d, other_vehicles);
      auto t = static_cast<double>(GetMissingPoints()) * kSampleDuration;
      auto next_s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t)
                  + GetFarthestPlannedS(nearest_s[0]) - nearest_s[0];
      auto new_planner_state = planner_state_->GetState(kNumberOfLanes,
                                                        kLaneWidth,
                                                        nearest_s, nearest_d,
                                                        kPreferredSpeed,
                                                        kTrajectoryTime,
                                                        next_s,
                                                        other_vehicles);
      if (new_planner_state) {
        std::cout << "Planner state changed" << std::endl;
        if (dynamic_cast<PlannerStateChangingLaneLeft*>(
              new_planner_state.get())
            || dynamic_cast<PlannerStateChangingLaneRight*>(
                 new_planner_state.get())) {
          // Perform the safe maneuver as soon as possible.
          if (previous_states_s_.size() > 10) {
            previous_states_s_.erase(previous_states_s_.begin() + 10,
                                     previous_states_s_.end());
            previous_states_d_.erase(previous_states_d_.begin() + 10,
                                     previous_states_d_.end());
          }
        }
        planner_state_ = new_planner_state;
      }
    }

    // Discard some planned points, if necessary.
    std::size_t n_reused_points = previous_states_s_.size();
    next_x.assign(previous_path_x.begin(),
                  previous_path_x.begin() + n_reused_points);
    next_y.assign(previous_path_y.begin(),
                  previous_path_y.begin() + n_reused_points);

    // Run full trajecrory generation.
    auto trajectory = GenerateTrajectory(current_d, other_vehicles);

    auto farthest_planned_s = GetFarthestPlannedS(nearest_s[0]);
    for (auto i = 1; i < GetMissingPoints() + 1; ++i) {
      auto t = static_cast<double>(i) * kSampleDuration;
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
      auto cartesian = coordinate_converter_.GetCartesian(nearest_s[0], {s, d});
      next_x.push_back(cartesian.x);
      next_y.push_back(cartesian.y);
    }
    std::cout << "Farthest planned s_dot " << previous_states_s_.back()[1]
              << std::endl;
  }

  if (!next_x.empty()) {
    // Control the simulator.
    control_function(next_x, next_y);
  }
}

// Private Methods
// -----------------------------------------------------------------------------

std::size_t PathPlanner::GetMissingPoints() const {
  return kNumberOfPathPoints - previous_states_s_.size();
}

double PathPlanner::GetPlanningTime() const {
  auto n_missing_points = kNumberOfPathPoints - previous_states_s_.size();
  return kPlanningTime - kTrajectoryTime + static_cast<double>(n_missing_points)
                                           * kSampleDuration;
}

double PathPlanner::GetFarthestPlannedS(double nearest_s) const {
  return !previous_states_s_.empty() ? previous_states_s_.back()[0] : nearest_s;
}

Vehicle::Trajectory PathPlanner::GenerateTrajectory(
  double current_d,
  const VehicleMap& other_vehicles) const {
  assert(planner_state_);
  Vehicle::Trajectory trajectory;

  auto target_vehicle_id = -1;
  std::size_t target_lane = 100;
  planner_state_->GetTarget(target_vehicle_id, target_lane);
  auto d = kLaneWidth * target_lane + kHalfLaneWidth;
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
  Vehicle::State target_s;
  Vehicle::State target_d = {d, 0, 0};

  std::cout << "Generating trajectory for planning_time " << planning_time
            << ", begin_s ("
            << begin_s[0] << "," << begin_s[1] << "," << begin_s[2] << ")"
            << ", begin_d ("
            << begin_d[0] << "," << begin_d[1] << "," << begin_d[2] << ")"
            << ", target_d ("
            << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")";
  auto target_vehicle = other_vehicles.find(target_vehicle_id);
  if (target_vehicle_id >= 0 && target_vehicle != other_vehicles.end()) {
    // Target vehicle is known, follow it.
    // TODO: Compute the planning time based on the speed and distance to other
    //       car.
    // f(x) = (x/(b*v^(2/3))-v^(1/3))^3+v, where v=20 and b=2
    // f(x) = (x/(b*v^(2/3))-v^(1/3)-v/(4*b*v^(2/3)))^3+v, where v=20 and b=2
    std::cout << ", target_vehicle_id " << target_vehicle_id;
    Vehicle::State target_vehicle_s;
    Vehicle::State target_vehicle_d;
    target_vehicle->second.GetState(0, target_vehicle_s, target_vehicle_d);
    std::cout << ", target_vehicle_s (" << target_vehicle_s[0] << ","
              << target_vehicle_s[1] << "," << target_vehicle_s[2] << ")";

    auto ds = target_vehicle_s[0];
    auto target_vehicle_speed = target_vehicle_s[1];
    std::cout << ", ds " << ds << ", target_vehicle_speed " << target_vehicle_speed;
    auto speed = std::pow(ds / (2. * std::pow(target_vehicle_speed, 2./3.))
                          - std::cbrt(target_vehicle_speed), 3)
                 + target_vehicle_speed;
    auto target_speed = std::min(speed, kPreferredSpeed);

    target_s = {target_speed * planning_time, target_speed,0};
    std::cout << ", target_s ("
              << target_s[0] << "," << target_s[1] << "," << target_s[2] << ")"
              << std::endl;
  } else {
    // Target vehicle is unknown, free run at comfortable speed.
    target_s = {kPreferredSpeed * planning_time, kPreferredSpeed, 0};
    std::cout << ", target_s ("
              << target_s[0] << "," << target_s[1] << "," << target_s[2] << ")"
              << std::endl;
  }
  return trajectory_generator_.Generate(begin_s, begin_d,
                                        target_s, target_d,
                                        planning_time,
                                        other_vehicles,
                                        kRoadWidth, kSpeedLimit);
}
