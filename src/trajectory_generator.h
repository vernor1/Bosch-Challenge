#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <random>
#include "vehicle.h"

class TrajectoryGenerator {
public:
  TrajectoryGenerator();

  Vehicle::Trajectory Generate(const Vehicle::State& begin_s,
                               const Vehicle::State& begin_d,
                               const Vehicle::State& target_s,
                               const Vehicle::State& target_d,
                               double target_time,
                               const VehicleMap& vehicles,
                               double d_limit,
                               double s_dot_limit) const;

  Vehicle::Trajectory Generate(const Vehicle::State& begin_s,
                               const Vehicle::State& begin_d,
                               std::size_t target_vehicle_id,
                               const Vehicle::State& delta_s,
                               const Vehicle::State& delta_d,
                               double target_time,
                               const VehicleMap& vehicles,
                               double d_limit,
                               double s_dot_limit) const;

private:
  std::random_device random_device_;
  mutable std::default_random_engine rng_;
  mutable std::normal_distribution<double> dist_s_;
  mutable std::normal_distribution<double> dist_s_dot_;
  mutable std::normal_distribution<double> dist_s_double_dot_;
  mutable std::normal_distribution<double> dist_d_;
  mutable std::normal_distribution<double> dist_d_dot_;
  mutable std::normal_distribution<double> dist_d_double_dot_;

  Vehicle::State PerturbS(const Vehicle::State& s) const;
  Vehicle::State PerturbD(const Vehicle::State& d) const;
};

#endif // TRAJECTORYGENERATOR_H
