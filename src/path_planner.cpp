#include "path_planner.h"
#include <cmath>
#include <iostream>
#include "spline.h"
#include "helpers.h"

// Local Types
// -----------------------------------------------------------------------------

// Local Constants
// -----------------------------------------------------------------------------

enum {N_PATH_POINTS = 50};

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Public Methods
// -----------------------------------------------------------------------------

PathPlanner::PathPlanner(const std::vector<double>& waypoints_x,
                         const std::vector<double>& waypoints_y,
                         const std::vector<double>& waypoints_dx,
                         const std::vector<double>& waypoints_dy,
                         const std::vector<double>& waypoints_s,
                         double max_s)
  : waypoints_x_(waypoints_x),
    waypoints_y_(waypoints_y),
    waypoints_s_(waypoints_s) { }

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
  std::vector<double> next_x(previous_path_x.begin(), previous_path_x.end());
  std::vector<double> next_y(previous_path_y.begin(), previous_path_y.end());

//  std::cout << "Previous path size " << previous_path_x.size()
//  << ", Cartesian coords";
//  for (std::size_t i = 0; i < previous_path_x.size(); ++i) {
//    std::cout << " (" << previous_path_x[i] << "," << previous_path_y[i] << ")";
//  }
//  std::cout << std::endl;
/*
  auto target_v = 20.1168;
  auto target_d = 6.;
  auto dt = 0.02;
  std::vector<double> next_x;
  std::vector<double> next_y;
  for (auto i = 1; i < 51; ++i) {
    auto target_s = current_s + static_cast<double>(i) * target_v * dt;
    auto cartesian = GetCartesian({target_s, target_d});
    next_x.push_back(cartesian.x);
    next_y.push_back(cartesian.y);
  }
  // Control the simulator.
  control_function(next_x, next_y);
*/

  if (!LastStateS.empty() && previous_path_x.empty()) {
    std::cerr << "Previous path exhausted!" << std::endl;
  }

  if (previous_path_x.size() < 25) {
    Vehicle::Trajectory trajectory;
    if (LastStateS.empty() || previous_path_x.empty()) {
      Vehicle::State begin_s = {current_s, 0, 0};
      Vehicle::State begin_d = {current_d, 0, 0};
      Vehicle::State target_s = {current_s + 60, 20, 0};
      Vehicle::State target_d = {2, 0, 0};
      trajectory = trajectory_generator_.Generate(begin_s,
                                                  begin_d,
                                                  target_s,
                                                  target_d,
                                                  3, VehicleMap());
    } else {
      Vehicle::State target_s = {LastStateS[0] + 60, 20, 0};
      Vehicle::State target_d = {2, 0, 0};
      trajectory = trajectory_generator_.Generate(LastStateS,
                                                  LastStateD,
                                                  target_s,
                                                  target_d,
                                                  3, VehicleMap());
    }
    auto dt = 0.02;
//    std::cout << "Current Frenet coords (" << current_s << "," << current_d
//              << "), Cartesian coords (" << current_x << "," << current_y
//              << "), trajectory time " << trajectory.time << std::endl;
//    std::cout << "Next Frenet coords";
    auto t = 0.;
    auto s = 0.;
    auto d = 0.;
    for (auto i = 1; i < 51; ++i) {
      t = static_cast<double>(i) * dt;
      s = helpers::EvaluatePolynomial(trajectory.s_coeffs, t);
      d = helpers::EvaluatePolynomial(trajectory.d_coeffs, t);
//      std::cout << " (" << s << "," << d << ")";
      auto cartesian = GetCartesian({s, d});
      next_x.push_back(cartesian.x);
      next_y.push_back(cartesian.y);
    }
//    std::cout << std::endl;

    auto s_dot = helpers::GetDerivative(trajectory.s_coeffs);
    auto s_double_dot = helpers::GetDerivative(s_dot);
    auto d_dot = helpers::GetDerivative(trajectory.d_coeffs);
    auto d_double_dot = helpers::GetDerivative(d_dot);
    LastStateS.clear();
    LastStateS.push_back(s);
    LastStateS.push_back(helpers::EvaluatePolynomial(s_dot, t));
    LastStateS.push_back(helpers::EvaluatePolynomial(s_double_dot, t));
    LastStateD.clear();
    LastStateD.push_back(d);
    LastStateD.push_back(helpers::EvaluatePolynomial(d_dot, t));
    LastStateD.push_back(helpers::EvaluatePolynomial(d_double_dot, t));
//    std::cout << "Last t " << t
//              << ", state_s (" << LastStateS[0] << "," << LastStateS[1] << ","
//              << LastStateS[2]
//              << "), state_d (" << LastStateD[0] << "," << LastStateD[1] << ","
//              << LastStateD[2] << ")" << std::endl;
/*
    std::cout << "Next Cartesian coords";
    for (std::size_t i = 0; i < next_x.size(); ++i) {
      std::cout << " (" << next_x[i] << "," << next_y[i] << ")";
    }
    std::cout << std::endl;
*/
  }

  if (!next_x.empty()) {
    // Control the simulator.
    control_function(next_x, next_y);
  }
}

// Private Methods
// -----------------------------------------------------------------------------

PathPlanner::Cartesian PathPlanner::GetCartesian(const Frenet& frenet) const {
  tk::spline spline_x;
  tk::spline spline_y;

  // TODO: Fit a spline over neighbour waypoints rather than whole track.
  spline_x.set_points(waypoints_s_, waypoints_x_);
  spline_y.set_points(waypoints_s_, waypoints_y_);

  auto x0 = spline_x(frenet.s);
  auto y0 = spline_y(frenet.s);

  auto norm_x = spline_y.deriv(1, frenet.s);
  auto norm_y = -spline_x.deriv(1, frenet.s);

  // FIXME: Handle the case of normal vector be shorter than the d-vector.
  assert(frenet.d * frenet.d > norm_x * norm_x - norm_y * norm_y);
  auto kd = std::sqrt(frenet.d * frenet.d - norm_x * norm_x - norm_y * norm_y);

  Cartesian cartesian({x0 + kd * norm_x, y0 + kd * norm_y});
/*
  std::cout << "GetCartesian: x " << cartesian.x << ", y " << cartesian.y
            << ", norm (" << norm_x << ", " << norm_y << ")"
            << ", kd " << kd
            << ", x0 " << x0 << ", y0 " << y0
            << std::endl;
*/
  return cartesian;
}
