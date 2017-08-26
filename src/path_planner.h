#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <deque>
#include <functional>
#include <vector>
#include "coordinate_converter.h"
#include "planner_state.h"
#include "trajectory_generator.h"

// Implements Path Planner.
// Embeds Coordinate Converter and Trajectory Generator.
// Aggregates Planner State.
class PathPlanner {
public:
  // Type of simulator control function.
  // @param[in] next_x  x-coordinates of next waypoints.
  // @param[in] next_y  y-coordinates of next waypoints.
  typedef std::function<void(
    const std::vector<double>& next_x,
    const std::vector<double>& next_y)> ControlFunction;

  // Type of detected vehicle as defined in Coordinate Converter.
  typedef CoordinateConverter::DetectedVehicle DetectedVehicle;

  // Constructor.
  // @param waypoints_x   Sequence of map waypoints, x-coordinate.
  // @param waypoints_y   Sequence of map waypoints, y-coordinate.
  // @param waypoints_s   Sequence of map waypoints, s-coordinate.
  // @param track_length  Track length [m].
  PathPlanner(const std::vector<double>& waypoints_x,
              const std::vector<double>& waypoints_y,
              const std::vector<double>& waypoints_s,
              double track_length);
  PathPlanner(const PathPlanner&) = delete;
  PathPlanner& operator=(const PathPlanner&) = delete;

  // Updates the Path Planner and controls the simulator.
  // @param[in] current_s         Current s-coordinate of the vehicle.
  // @param[in] current_d         Current d-coordinate of the vehicle.
  // @param[in] previous_path_x   Sequence of x-coordinates not yet passed.
  // @param[in] previous_path_y   Sequence of y-coordinates not yet passed.
  // @param[in] sensor_fusion     Detected vehicles.
  // @param[in] lane_width        Lane width [m].
  // @param[in] n_lanes           Number of lanes [m].
  // @param[in] speed_limit       Speed limit [m/s].
  // @param[in] control_function  Simulator control function.
  void Update(double current_s,
              double current_d,
              double current_x,
              double current_y,
              const std::vector<double>& previous_path_x,
              const std::vector<double>& previous_path_y,
              const std::vector<DetectedVehicle>& sensor_fusion,
              double lane_width,
              std::size_t n_lanes,
              double speed_limit,
              ControlFunction control_function);

private:
  CoordinateConverter coordinate_converter_;
  TrajectoryGenerator trajectory_generator_;
  std::shared_ptr<PlannerState> planner_state_;
  std::deque<Vehicle::State> previous_states_s_;
  std::deque<Vehicle::State> previous_states_d_;
  std::size_t n_remaining_planned_points_;
  bool is_initial_offset_computed_;
  double initial_offset_x_;
  double initial_offset_y_;

  inline std::size_t GetMissingPoints() const;
  inline double GetPlanningTime() const;
  inline Vehicle::State GetNearestS(double current_s) const;
  inline Vehicle::State GetNearestD(double current_d) const;
  inline double GetFarthestPlannedS(double current_s) const;
  inline void DiscardPreviousStates();
  inline void GetTrajectoryBegin(double current_d,
                                 Vehicle::State& begin_s,
                                 Vehicle::State& begin_d) const;

  // Generates the trajectory.
  Vehicle::Trajectory GenerateTrajectory(double current_d,
                                         double lane_width,
                                         std::size_t n_lanes,
                                         double speed_limit,
                                         double preferred_speed,
                                         const VehicleMap& other_vehicles);

  // Generates new next points and update previous states.
  void AddNextPoints(const Vehicle::Trajectory& trajectory,
                     double current_s,
                     double current_x,
                     double current_y,
                     std::vector<double>& next_x,
                     std::vector<double>& next_y);
};

#endif // PATHPLANNER_H
