#include "path_planner.h"
#include <iostream>
#include "helpers.h"

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Number of points in the projected path.
enum {kNumberOfPathPoints = 50};

// Sample duration of the projected path.
const auto kSampleDuration = 0.02;

// Preferred speed offset below the speed limit [m/s].
const auto kPreferredBufferSpeed = 2.5 * helpers::kMphToMps;

// Preferred buffer time when following other vehicles [s].
const auto kPreferredBufferTime = 3.;

// Preferred longitudinal acceleration [m/s/s].
const auto kPreferredLongAccel = 1.;

// Preferred lateral acceleration [m/s/s].
const auto kPreferredLatAccel = 5.;

// Trajectory planning time [s].
const auto kPlanningTime = 2.;

// Projected trajectory time [s].
const auto kTrajectoryTime = 1.;

// Number of waypoints to discard.
enum {kNumberOfPointsToDiscard = 5};

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

PathPlanner::PathPlanner(const std::vector<double>& waypoints_x,
                         const std::vector<double>& waypoints_y,
                         const std::vector<double>& waypoints_s)
  : coordinate_converter_(waypoints_x, waypoints_y, waypoints_s),
    n_remaining_planned_points_(),
    is_initial_offset_computed_() {
  // Empty.
}

void PathPlanner::Update(double current_s,
                         double current_d,
                         double current_x,
                         double current_y,
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
    //std::cout << "Nearest s (" << nearest_s[0] << "," << nearest_s[1] << ","
    //          << nearest_s[2] << ")"
    //          << ", d (" << nearest_d[0] << "," << nearest_d[1] << ","
    //          << nearest_d[2] << ")"
    //          << std::endl;
    auto other_vehicles = coordinate_converter_.GetVehicles(nearest_s[0],
                                                            sensor_fusion);

    const auto adjusted_speed_limit = speed_limit - kPreferredBufferSpeed;
    const auto preferred_speed = adjusted_speed_limit - kPreferredBufferSpeed;
    if (n_remaining_planned_points_ == 0) {
      // Determine next planner state.
      std::cout << "Determine next planner state" << std::endl;
      n_remaining_planned_points_ = kNumberOfPathPoints;
      if (!planner_state_) {
        planner_state_.reset(new PlannerStateKeepingLane(1));
      }
      // Dry-run the trajectory generator to determine next s and d.
      auto trajectory = GenerateTrajectory(current_d, lane_width, n_lanes,
                                           adjusted_speed_limit,
                                           preferred_speed, other_vehicles);
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
                                         adjusted_speed_limit, preferred_speed,
                                         other_vehicles);
    //std::cout << "Trajectory s_coeffs ("
    //          << trajectory.s_coeffs[0] << "," << trajectory.s_coeffs[1]
    //          << "," << trajectory.s_coeffs[2] << "," << trajectory.s_coeffs[3]
    //          << "," << trajectory.s_coeffs[4] << "," << trajectory.s_coeffs[5]
    //          << "), d_coeffs ("
    //          << trajectory.d_coeffs[0] << "," << trajectory.d_coeffs[1]
    //          << "," << trajectory.d_coeffs[2] << "," << trajectory.d_coeffs[3]
    //          << "," << trajectory.d_coeffs[4] << "," << trajectory.d_coeffs[5]
    //          << ")" << std::endl;

    if (n_remaining_planned_points_ != kNumberOfPathPoints) {
      // If whis is not a planning cycle, update the target vehicle Id.
      auto planning_time = GetPlanningTime();
      auto next_s = helpers::EvaluatePolynomial(trajectory.s_coeffs,
                                                planning_time);
      planner_state_->UpdateTargetVehicleId(planning_time,
                                            next_s,
                                            other_vehicles);
    }

    // Reuse next points.
    std::size_t n_reused_points = previous_states_s_.size();
    next_x.assign(previous_path_x.begin(),
                  previous_path_x.begin() + n_reused_points);
    next_y.assign(previous_path_y.begin(),
                  previous_path_y.begin() + n_reused_points);

    // Add missing next points.
    AddNextPoints(trajectory, current_s, current_x, current_y, next_x, next_y);
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

double PathPlanner::GetFarthestPlannedS(double current_s) const {
  return !previous_states_s_.empty() ? previous_states_s_.back()[0] : current_s;
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

  auto planning_time = GetPlanningTime();
  auto target_vehicle = other_vehicles.find(target_vehicle_id);
  Vehicle::State begin_s;
  Vehicle::State begin_d;
  auto feasible_s_dot = 0.;
  std::cout << "Generating trajectory";
  if (target_vehicle_id >= 0 && target_vehicle != other_vehicles.end()) {
    // Target vehicle is known, follow it.
    // There are two issues, which prevent using the trajectory generation
    // function for following other vehicle:
    //   1) Coordinates of other vehicles are not precise, they're taken from
    //      the simulator, while the local coordinate system is computed off
    //      splines.
    //   2) Simulation of other vehicles is imperfect: they drive erraticaly
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
    auto target_speed = preferred_speed;
    if (ds < preferred_speed * kPreferredBufferTime) {
      auto target_vehicle_speed = target_vehicle_s0[1];
      std::cout << ", target_vehicle_speed " << target_vehicle_speed;
      auto speed = std::pow(
        ds / (kPreferredBufferTime * std::pow(target_vehicle_speed, 2./3.))
        - std::cbrt(target_vehicle_speed), 3) + target_vehicle_speed;
      target_speed = std::min(speed, preferred_speed);
      // Disacard previous states to be able to react on sudden speed
      // changes of the other vehicle.
      DiscardPreviousStates();
      planning_time = GetPlanningTime();
    }
    GetTrajectoryBegin(current_d, begin_s, begin_d);
    // If slower that preferred speed, increase it with the preferred
    // acceleration.
    feasible_s_dot = std::min(target_speed,
                              begin_s[1] + kPreferredLongAccel * planning_time);
  } else {
    // Target vehicle is unknown, free run at comfortable speed.
    GetTrajectoryBegin(current_d, begin_s, begin_d);
    // If slower that preferred speed, increase it with the preferred
    // acceleration.
    feasible_s_dot = std::min(preferred_speed,
                              begin_s[1] + kPreferredLongAccel * planning_time);
  }
  Vehicle::State target_s = {feasible_s_dot * planning_time, feasible_s_dot, 0};
  auto d = lane_width * (target_lane + 0.5);

  // Adjust d to a feasible travel distance when in the free running mode.
  auto feasible_dd = kPreferredLatAccel * planning_time * planning_time / 2.;
  if (feasible_dd < std::fabs(d - begin_d[0])) {
    std::cout << ", adjusted d " << d;
    d = d > begin_d[0] ? begin_d[0] + feasible_dd : begin_d[0] - feasible_dd;
    std::cout << " -> " << d;
  }

  Vehicle::State target_d = {d, 0, 0};
  std::cout << ", begin_s ("
            << begin_s[0] << "," << begin_s[1] << "," << begin_s[2] << ")"
            << ", begin_d ("
            << begin_d[0] << "," << begin_d[1] << "," << begin_d[2] << ")"
            << ", target_s ("
            << target_s[0] << "," << target_s[1] << "," << target_s[2] << ")"
            << ", target_d ("
            << target_d[0] << "," << target_d[1] << "," << target_d[2] << ")"
            << ", planning_time " << planning_time << std::endl;
  return trajectory_generator_.Generate(
    begin_s, begin_d, target_s, target_d, planning_time, other_vehicles,
    static_cast<double>(n_lanes) * lane_width, speed_limit);
}

void PathPlanner::AddNextPoints(const Vehicle::Trajectory& trajectory,
                                double current_s,
                                double current_x,
                                double current_y,
                                std::vector<double>& next_x,
                                std::vector<double>& next_y) {
  auto s_dot_coeffs = helpers::GetDerivative(trajectory.s_coeffs);
  auto s_double_dot_coeffs = helpers::GetDerivative(s_dot_coeffs);
  auto d_dot_coeffs = helpers::GetDerivative(trajectory.d_coeffs);
  auto d_double_dot_coeffs = helpers::GetDerivative(d_dot_coeffs);
  auto n_missing_points = GetMissingPoints();
  auto farthest_planned_s = GetFarthestPlannedS(current_s);
  if (!is_initial_offset_computed_) {
    auto s = helpers::EvaluatePolynomial(trajectory.s_coeffs, 0)
           + farthest_planned_s;
    auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, 0);
    auto cartesian = coordinate_converter_.GetCartesian(current_s, {s, d});
    initial_offset_x_ = current_x - cartesian.x;
    initial_offset_y_ = current_y - cartesian.y;
    is_initial_offset_computed_ = true;
    std::cout << "Current coords (" << current_x << "," << current_y
              << "), spline (" << cartesian.x << "," << cartesian.y
              << "), offset (" << initial_offset_x_ << ","
              << initial_offset_y_ << ")" << std::endl;
  }
  for (auto i = 1; i < n_missing_points + 1; ++i) {
    auto t = static_cast<double>(i) * kSampleDuration;
    auto s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t)
           + farthest_planned_s;
    auto d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
    auto cartesian = coordinate_converter_.GetCartesian(current_s, {s, d});
    auto s_dot = helpers::EvaluatePolynomial(s_dot_coeffs, t);
    auto d_dot = helpers::EvaluatePolynomial(d_dot_coeffs, t);
    auto s_double_dot = helpers::EvaluatePolynomial(s_double_dot_coeffs, t);
    auto d_double_dot = helpers::EvaluatePolynomial(d_double_dot_coeffs, t);
    previous_states_s_.push_back({s, s_dot, s_double_dot});
    previous_states_d_.push_back({d, d_dot, d_double_dot});
    //std::cout << "New s (" << s << "," << s_dot << "," << s_double_dot << ")"
    //          << ", d (" << d << "," << d_dot << "," << d_double_dot << ")"
    //          << ", x " << cartesian.x + initial_offset_x_
    //          << ", y " << cartesian.y + initial_offset_y_
    //          << std::endl;
    next_x.push_back(cartesian.x + initial_offset_x_);
    next_y.push_back(cartesian.y + initial_offset_y_);
  }
}
