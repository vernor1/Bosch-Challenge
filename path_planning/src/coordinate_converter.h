#ifndef COORDINATE_CONVERTER_H
#define COORDINATE_CONVERTER_H

#include <map>
#include "spline.h"
#include "vehicle.h"

// Implements Coordinate Converter.
class CoordinateConverter {
public:
  // Type of Frenet coords.
  struct Frenet {
    double s;
    double d;
  };

  // Type of Cartesian coords.
  struct Cartesian {
    double x;
    double y;
  };

  // Type of a detected vehicle.
  struct DetectedVehicle {
    std::size_t id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
  };

  // Constructor.
  // @param waypoints_x   Sequence of map waypoints, x-coordinate.
  // @param waypoints_x   Sequence of map waypoints, y-coordinate.
  // @param waypoints_x   Sequence of map waypoints, s-coordinate.
  // @param track_length  Track length [m].
  CoordinateConverter(const std::vector<double>& waypoints_x,
                      const std::vector<double>& waypoints_y,
                      const std::vector<double>& waypoints_s,
                      double track_length);
  CoordinateConverter(const CoordinateConverter&) = delete;
  CoordinateConverter& operator=(const CoordinateConverter&) = delete;

  // Converts Frenet to Cartesian.
  // @param[in] current_s  Current s-coordinate.
  // @param[in] frenet     Frenet coordinates.
  // @return  Cartesian coordinates.
  Cartesian GetCartesian(double current_s, const Frenet& frenet);

  // Converts other vehicles' coordinates from the instant simulator format
  // to Frenet state coordinates.
  // @param[in] current_s      Current s-coordinate.
  // @param[in] sensor_fusion  Detected vehicles.
  // @return  Other vehicle map.
  VehicleMap GetVehicles(double current_s,
                         const std::vector<DetectedVehicle>& sensor_fusion);

private:
  struct CartesianWaypoint {
    std::size_t id;
    double x;
    double y;
  };

  double track_length_;
  double half_track_length_;
  std::map<double, CartesianWaypoint> waypoints_map_;
  std::vector<std::size_t> current_waypoints_id_;
  tk::spline spline_x_;
  tk::spline spline_y_;

  // Updates current splines in necessary.
  void UpdateSplines(double current_s);
};

#endif // COORDINATE_CONVERTER_H
