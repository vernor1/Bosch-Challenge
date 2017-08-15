#include "path_planner.h"
#include <iostream>
#include "helpers.h"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Number of points in the projected path.
enum {kNumberOfPathPoints = 50};

// Sample duration of the projected path.
auto kSampleDuration = 0.02;

// Preferred speed offset below the speed limit [m/s]
const auto kPreferredBufferSpeed = 5. * helpers::kMphToMps;

// Preferred buffer time when following other vehicles [s].
auto kPreferredBufferTime = 3.;

// Trajectory planning time [s].
auto kPlanningTime = 2.;

// Projected trajectory time [s].
auto kTrajectoryTime = 1.;

// Number of waypoints to discard.
enum {kNumberOfPointsToDiscard = 5};

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

PathPlanner::PathPlanner(const std::vector<double>& waypoints_x,
                         const std::vector<double>& waypoints_y,
                         const std::vector<double>& waypoints_s,
                         double track_length)
  : coordinate_converter_(waypoints_x, waypoints_y, waypoints_s, track_length),
    n_remaining_planned_points_() {
  // Empty.
}

void PathPlanner::Update(double current_s,
                         double current_d,
                         const std::vector<double>& previous_path_x,
                         const std::vector<double>& previous_path_y,
                         const std::vector<DetectedVehicle>& sensor_fusion,
                         double lane_width,
                         std::size_t n_lanes,
                         double speed_limit,
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

    auto nearest_s = GetNearestS(current_s);
    auto nearest_d = GetNearestD(current_d);
    auto other_vehicles = coordinate_converter_.GetVehicles(nearest_s[0],
                                                            sensor_fusion);

    const auto preferred_speed = speed_limit - kPreferredBufferSpeed;
    if (n_remaining_planned_points_ == 0) {
      // Determine next planner state.
      std::cout << "Determine next planner state" << std::endl;
      n_remaining_planned_points_ = kNumberOfPathPoints;
      if (!planner_state_) {
        planner_state_.reset(new PlannerStateKeepingLane(1));
      }
      // Dry-run the trajectory generator to determine next s and d.
      auto trajectory = GenerateTrajectory(current_d, lane_width, n_lanes,
                                           speed_limit, preferred_speed,
                                           other_vehicles);
      auto planning_time = GetPlanningTime();
      auto next_s = helpers::EvaluatePolynomial(trajectory.s_coeffs,
                                                planning_time);
      auto new_planner_state = planner_state_->GetState(n_lanes,
                                                        lane_width,
                                                        nearest_s, nearest_d,
                                                        preferred_speed,
                                                        planning_time,
                                                        next_s,
                                                        other_vehicles);
      if (new_planner_state) {
        std::cout << "Planner state changed" << std::endl;
        if (dynamic_cast<PlannerStateChangingLaneLeft*>(
              new_planner_state.get())
            || dynamic_cast<PlannerStateChangingLaneRight*>(
                 new_planner_state.get())) {
          // Perform the safe maneuver as soon as possible.
          DiscardPreviousStates();
        }
        planner_state_ = new_planner_state;
      }
    }

    // Run full trajecrory generation.
    auto trajectory = GenerateTrajectory(current_d, lane_width, n_lanes,
                                         speed_limit, preferred_speed,
                                         other_vehicles);

    // Reuse next points.
    std::size_t n_reused_points = previous_states_s_.size();
    next_x.assign(previous_path_x.begin(),
                  previous_path_x.begin() + n_reused_points);
    next_y.assign(previous_path_y.begin(),
                  previous_path_y.begin() + n_reused_points);

    // Add missing next points.
    AddNextPoints(trajectory, nearest_s, next_x, next_y);
    auto farthest_planned_s = previous_states_s_.back();
    std::cout << "Farthest planned s (" << farthest_planned_s[0] << ","
              << farthest_planned_s[1] << "," << farthest_planned_s[2] << ")"
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
  return kPlanningTime - kTrajectoryTime
         + static_cast<double>(GetMissingPoints()) * kSampleDuration;
}

Vehicle::State PathPlanner::GetNearestS(double current_s) const {
  return !previous_states_s_.empty()
         ? previous_states_s_.front()
         : Vehicle::State{current_s, 0, 0};
}

Vehicle::State PathPlanner::GetNearestD(double current_d) const {
  return !previous_states_d_.empty()
         ? previous_states_d_.front()
         : Vehicle::State{current_d, 0, 0};
}

double PathPlanner::GetFarthestPlannedS(double nearest_s) const {
  return !previous_states_s_.empty() ? previous_states_s_.back()[0] : nearest_s;
}

void PathPlanner::DiscardPreviousStates() {
  if (previous_states_s_.size() > kNumberOfPointsToDiscard) {
    previous_states_s_.erase(
      previous_states_s_.begin() + kNumberOfPointsToDiscard,
      previous_states_s_.end());
    previous_states_d_.erase(
      previous_states_d_.begin() + kNumberOfPointsToDiscard,
      previous_states_d_.end());
  }
}

void PathPlanner::GetTrajectoryBegin(double current_d,
                                     Vehicle::State& begin_s,
                                     Vehicle::State& begin_d) const {
  if (!previous_states_s_.empty()) {
    begin_s = previous_states_s_.back();
    begin_s[0] = 0;
    begin_d = previous_states_d_.back();
  } else {
    begin_s = {0, 0, 0};
    begin_d = {current_d, 0, 0};
  }
}

Vehicle::Trajectory PathPlanner::GenerateTrajectory(
  double current_d,
  double lane_width,
  std::size_t n_lanes,
  double speed_limit,
  double preferred_speed,
  const VehicleMap& other_vehicles) {
  assert(planner_state_);
  Vehicle::Trajectory trajectory;

  auto target_vehicle_id = -1;
  std::size_t target_lane = 100;
  planner_state_->GetTarget(target_vehicle_id, target_lane);
  auto d = lane_width * (target_lane + 0.5);

  auto planning_time = GetPlanningTime();
  auto target_vehicle = other_vehicles.find(target_vehicle_id);
  Vehicle::State begin_s;
  Vehicle::State begin_d;
  Vehicle::State target_s;
  std::cout << "Generating trajectory";
  if (target_vehicle_id >= 0 && target_vehicle != other_vehicles.end()) {
    // Target vehicle is known, follow it.
    // There are two issues, which prevent using the tragectory generation
    // function for following other vehicle:
    //   1) Coordinates of other vehicles are not precise, they're taken from
    //      the simulator, while the local coordinate system is computed off
    //      splines.
    //   2) Simulation of other vehicles is imperhect: they drive erraticaly
    //      with extreme jerks when the're following other vehicles.
    // So own speed is computed as a function of distance to other vehicle (ds)
    // and its speed (v): f(ds,v) = (ds/(b*v^(2/3))-v^(1/3))^3+v,
    // where v is the other vehicle's speed,
    //       b is the buffer time.
    std::cout << ", target_vehicle_id " << target_vehicle_id;
    Vehicle::State target_vehicle_s0;
    Vehicle::State target_vehicle_d0;
    target_vehicle->second.GetState(0, target_vehicle_s0, target_vehicle_d0);
    std::cout << ", target_vehicle_s0 (" << target_vehicle_s0[0] << ","
              << target_vehicle_s0[1] << "," << target_vehicle_s0[2] << ")";

    auto ds = target_vehicle_s0[0];
    std::cout << ", ds " << ds;
    auto target_speed = preferred_speed;
    if (ds < preferred_speed * kPreferredBufferTime) {
      auto target_vehicle_speed = target_vehicle_s0[1];
      std::cout << ", target_vehicle_speed " << target_vehicle_speed;
      std::cout << ", ds " << ds << ", target_vehicle_speed " << target_vehicle_speed;
      auto speed = std::pow(
        ds / (kPreferredBufferTime * std::pow(target_vehicle_speed, 2./3.))
        - std::cbrt(target_vehicle_speed), 3) + target_vehicle_speed;
      target_speed = std::min(speed, preferred_speed);
      // Disacard previous states to be able to react on sudden speed
      // changes of the other vehicle.
      DiscardPreviousStates();
      planning_time = GetPlanningTime();
    }
    target_s = {target_speed * planning_time, target_speed, 0};
    GetTrajectoryBegin(current_d, begin_s, begin_d);
  } else {
    // Target vehicle is unknown, free run at comfortable speed.
    GetTrajectoryBegin(current_d, begin_s, begin_d);
    // If slower that preferred speed, increase it at a rate of 1 m/s/s.
    auto feasible_target_speed = std::min(preferred_speed,
                                          begin_s[1] + planning_time);
    target_s = {feasible_target_speed * planning_time, feasible_target_speed, 0};
  }

  Vehicle::State target_d = {d, 0, 0};
  std::cout << ", begin_s ("
            << begin_s[0] << "," << begin_s[1] << "," << begin_s[2] << ")"
            << ", begin_d ("
            << begin_d[0] << "," << begin_d[1] << "," << begin_d[2] << ")"
            << ", target_s ("
            << target_s[0] << "," << target_s[1] << "," << target_s[2]
            << ", target_d ("
            << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")"
            << "), planning_time " << planning_time << std::endl;
  return trajectory_generator_.Generate(
    begin_s, begin_d, target_s, target_d, planning_time, other_vehicles,
    static_cast<double>(n_lanes) * lane_width, speed_limit);
}

void PathPlanner::AddNextPoints(const Vehicle::Trajectory& trajectory,
                                const Vehicle::State& nearest_s,
                                std::vector<double>& next_x,
                                std::vector<double>& next_y) {
  auto n_missing_points = GetMissingPoints();
  auto farthest_planned_s = GetFarthestPlannedS(nearest_s[0]);
  for (auto i = 1; i < n_missing_points + 1; ++i) {
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
}
