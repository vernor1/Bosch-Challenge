#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <deque>
#include <functional>
#include <vector>
#include "trajectory_generator.h"

class PathPlanner {
public:
  typedef std::function<void(
    const std::vector<double>& next_x,
    const std::vector<double>& next_y)> ControlFunction;

  // [id, x, y, vx, vy, s, d]
  struct DetectedVehicle{
    std::size_t id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
  };

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
  // TODO: Consider removing structures Frenet and Cartesian.
  struct Frenet {
    double s;
    double d;
  };
  struct Cartesian {
    double x;
    double y;
  };

  std::vector<double> waypoints_x_;
  std::vector<double> waypoints_y_;
  std::vector<double> waypoints_s_;
  TrajectoryGenerator trajectory_generator_;
//  Vehicle::State LastStateS;
//  Vehicle::State LastStateD;
  std::deque<Vehicle::State> previous_states_s_;
  std::deque<Vehicle::State> previous_states_d_;

  Cartesian GetCartesian(const Frenet& frenet) const;
};

#endif // PATHPLANNER_H
