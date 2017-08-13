#ifndef COORDINATE_CONVERTER_H
#define COORDINATE_CONVERTER_H

#include <map>
#include "spline.h"
#include "vehicle.h"

class CoordinateConverter {
public:
  // TODO: Consider removing structures Frenet and Cartesian.
  struct Frenet {
    double s;
    double d;
  };
  struct Cartesian {
    double x;
    double y;
  };

  // [id, x, y, vx, vy, s, d]
  struct DetectedVehicle {
    std::size_t id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
  };

  CoordinateConverter(const std::vector<double>& waypoints_x,
                      const std::vector<double>& waypoints_y,
                      const std::vector<double>& waypoints_s,
                      double max_s);

  Cartesian GetCartesian(const Frenet& frenet) const;

  VehicleMap GetVehicles(double current_s,
                         const std::vector<DetectedVehicle>& sensor_fusion);

private:
  std::vector<double> waypoints_x_;
  std::vector<double> waypoints_y_;
  std::vector<double> waypoints_s_;
  std::map<double, Cartesian> waypoints_map_;
  tk::spline spline_x_;
  tk::spline spline_y_;

  void UpdateSplines(double current_s);
};

#endif // COORDINATE_CONVERTER_H
