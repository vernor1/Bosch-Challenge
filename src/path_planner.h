#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <deque>
#include <functional>
#include <vector>
#include "coordinate_converter.h"
#include "planner_state.h"
#include "trajectory_generator.h"

class PathPlanner {
public:
  typedef std::function<void(
    const std::vector<double>& next_x,
    const std::vector<double>& next_y)> ControlFunction;

  typedef CoordinateConverter::DetectedVehicle DetectedVehicle;

  PathPlanner(const std::vector<double>& waypoints_x,
              const std::vector<double>& waypoints_y,
              const std::vector<double>& waypoints_dx,
              const std::vector<double>& waypoints_dy,
              const std::vector<double>& waypoints_s,
              double max_s);

  void Update(double current_x,
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
              ControlFunction control_function);

private:
  CoordinateConverter coordinate_converter_;
  TrajectoryGenerator trajectory_generator_;
  std::shared_ptr<PlannerState> planner_state_;
  std::deque<Vehicle::State> previous_states_s_;
  std::deque<Vehicle::State> previous_states_d_;
  std::size_t n_remaining_planned_points_;

  std::size_t GetMissingPoints() const;
  double GetPlanningTime() const;
  Vehicle::Trajectory GenerateTrajectory(
    double current_d,
    const VehicleMap& other_vehicles) const;
};

#endif // PATHPLANNER_H
