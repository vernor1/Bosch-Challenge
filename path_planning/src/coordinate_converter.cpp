#include "coordinate_converter.h"
#include <cmath>
#include <iostream>

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// Observed sensing range [m]
auto kSensingRange = 300.;

} // namespace

// Public Methods
// -----------------------------------------------------------------------------

CoordinateConverter::CoordinateConverter(const std::vector<double>& waypoints_x,
                                         const std::vector<double>& waypoints_y,
                                         const std::vector<double>& waypoints_s,
                                         double track_length)
  : track_length_(track_length),
    half_track_length_(track_length / 2.) {
  assert(waypoints_x.size() == waypoints_s.size());
  assert(waypoints_y.size() == waypoints_s.size());
  for (std::size_t i = 0; i < waypoints_s.size(); ++i) {
    waypoints_map_.insert(
      std::make_pair(waypoints_s[i],
                     CartesianWaypoint{i, waypoints_x[i], waypoints_y[i]}));
  }
}

CoordinateConverter::Cartesian CoordinateConverter::GetCartesian(
  double current_s,
  const Frenet& frenet) {

  UpdateSplines(current_s);

  auto x0 = spline_x_(frenet.s);
  auto y0 = spline_y_(frenet.s);

  auto norm_x = spline_y_.deriv(1, frenet.s);
  auto norm_y = -spline_x_.deriv(1, frenet.s);
  assert(frenet.d * frenet.d > norm_x * norm_x - norm_y * norm_y);

  auto kd = std::sqrt(frenet.d * frenet.d - norm_x * norm_x - norm_y * norm_y);

  return {x0 + kd * norm_x, y0 + kd * norm_y};
}

VehicleMap CoordinateConverter::GetVehicles(
  double current_s,
  const std::vector<DetectedVehicle>& sensor_fusion) {

  UpdateSplines(current_s);

  VehicleMap vehicles;
  for (const auto& sf : sensor_fusion) {
    // NOTE: The simulator app returns negative d-coords in the beginning of
    //       simulation.
    if (sf.d > 0) {
      auto vehicle_s = sf.s;
      if (std::fabs(current_s - vehicle_s) > half_track_length_) {
        // The simulator app returns other car's s-coord relative to each track
        // start, while current_s is is relative to the start of first track.
        auto current_s_remainder = std::fmod(current_s, track_length_);
        auto current_s_quotient = std::floor(current_s / track_length_);
        auto vehicle_s_remainder = std::fmod(vehicle_s, track_length_);
        if (std::fabs(current_s_remainder - vehicle_s_remainder)
            < half_track_length_) {
          vehicle_s += track_length_ * current_s_quotient;
        } else if (current_s_remainder > vehicle_s_remainder) {
          vehicle_s += track_length_ * (current_s_quotient + 1);
        } else {
          vehicle_s += track_length_ * (current_s_quotient - 1);
        }
      }

      auto cartesian_yaw = std::atan2(sf.vy, sf.vx);
      auto x0 = spline_x_(vehicle_s);
      auto y0 = spline_y_(vehicle_s);
      auto x1 = spline_x_(vehicle_s + 1);
      auto y1 = spline_y_(vehicle_s + 1);
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
        Vehicle({vehicle_s - current_s, vs, 0}, {sf.d, vd, 0})));
    }
  }
  return vehicles;
}

// Private Methods
// -----------------------------------------------------------------------------
void CoordinateConverter::UpdateSplines(double current_s) {
  auto range_begin = current_s - kSensingRange;
  auto range_begin_quotient = std::floor(range_begin / track_length_);
  auto range_begin_remainder = std::fmod(range_begin, track_length_);

  auto range_end = current_s + kSensingRange;
  auto range_end_quotient = std::floor(range_end / track_length_);
  auto range_end_remainder = std::fmod(range_end, track_length_);

  if (range_begin_remainder < 0) {
    range_begin_remainder += track_length_;
  }
  std::vector<std::size_t> waypoints_id;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  auto iter = waypoints_map_.upper_bound(range_begin_remainder);
  if (range_begin_quotient != range_end_quotient) {
    while (iter != waypoints_map_.end()) {
      waypoints_s.push_back(iter->first + track_length_ * range_begin_quotient);
      waypoints_id.push_back(iter->second.id);
      waypoints_x.push_back(iter->second.x);
      waypoints_y.push_back(iter->second.y);
      ++iter;
    }
    iter = waypoints_map_.begin();
  }
  while (iter != waypoints_map_.end() && iter->first < range_end_remainder) {
    waypoints_s.push_back(iter->first + track_length_ * range_end_quotient);
    waypoints_id.push_back(iter->second.id);
    waypoints_x.push_back(iter->second.x);
    waypoints_y.push_back(iter->second.y);
    ++iter;
  }

  if (current_waypoints_id_ != waypoints_id) {
    current_waypoints_id_ = waypoints_id;
    spline_x_.set_points(waypoints_s, waypoints_x);
    spline_y_.set_points(waypoints_s, waypoints_y);
    std::cout << "Updated splines" << std::endl;
  }
}
