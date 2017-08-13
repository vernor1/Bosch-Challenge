#include "coordinate_converter.h"
#include <cmath>
#include <iostream>

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Public Methods
// -----------------------------------------------------------------------------

CoordinateConverter::CoordinateConverter(const std::vector<double>& waypoints_x,
                                         const std::vector<double>& waypoints_y,
                                         const std::vector<double>& waypoints_s,
                                         double max_s)
  : waypoints_x_(waypoints_x),
    waypoints_y_(waypoints_y),
    waypoints_s_(waypoints_s) {
  assert(waypoints_x.size() == waypoints_s.size());
  assert(waypoints_y.size() == waypoints_s.size());
  for (std::size_t i = 0; i < waypoints_s.size(); ++i) {
    waypoints_map_.insert(
      std::make_pair(waypoints_s[i],
                     Cartesian{waypoints_x[i], waypoints_y[i]}));
  }

  spline_x_.set_points(waypoints_s, waypoints_x);
  spline_y_.set_points(waypoints_s, waypoints_y);
}

CoordinateConverter::Cartesian CoordinateConverter::GetCartesian(
  const Frenet& frenet) const {
//  tk::spline spline_x;
//  tk::spline spline_y;

  // TODO: Fit a spline over neighbour waypoints rather than whole track.
//  spline_x_.set_points(waypoints_s_, waypoints_x_);
//  spline_y_.set_points(waypoints_s_, waypoints_y_);

  auto x0 = spline_x_(frenet.s);
  auto y0 = spline_y_(frenet.s);

  auto norm_x = spline_y_.deriv(1, frenet.s);
  auto norm_y = -spline_x_.deriv(1, frenet.s);

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

VehicleMap CoordinateConverter::GetVehicles(
  double current_s,
  const std::vector<DetectedVehicle>& sensor_fusion) {

  UpdateSplines(current_s);

  VehicleMap vehicles;
  for (const auto& sf : sensor_fusion) {
    if (sf.s > 0 && sf.d > 0) {
      auto cartesian_yaw = std::atan2(sf.vy, sf.vx);
      auto x0 = spline_x_(sf.s);
      auto y0 = spline_y_(sf.s);
      auto x1 = spline_x_(sf.s + 1);
      auto y1 = spline_y_(sf.s + 1);
      auto lane_orientation = std::atan2(y1 - y0, x1 - x0);
      auto frenet_yaw = cartesian_yaw - lane_orientation;
      auto cartesian_v = std::sqrt(sf.vx * sf.vx + sf.vy * sf.vy);
      auto vs = cartesian_v * std::cos(frenet_yaw);
      auto vd = cartesian_v * std::sin(frenet_yaw);
      // NOTE: This simplistic conversion provides a rough estimate of the other
      // car's s-coordinate, which cannot be directly used for following the
      // car. This is because the current s-coord as well as the sensor fusion
      // s-coords are provided by the simulator, which obtained them in an
      // unknown way (perhaps by linear interpolation), while the own s,d-coords
      // are computed out from the spline fitted over map coordinates. A better
      // solution would be using sensor fusion x,y-coords and compute other
      // car'ss,d-coords out from the fitted splines.
      vehicles.insert(std::make_pair(
        sf.id,
        Vehicle({sf.s - current_s, vs, 0}, {sf.d, vd, 0})));
    }
  }
  return vehicles;
}

// Private Methods
// -----------------------------------------------------------------------------
void CoordinateConverter::UpdateSplines(double current_s) {
  std::cout << "current_s " << current_s << ", upper_bound ";
  auto ub = waypoints_map_.upper_bound(current_s);
  if (ub != waypoints_map_.end()) {
    std::cout << ub->first;
  } else {
    std::cout << "not determined";
  }
  std::cout << std::endl;
}
