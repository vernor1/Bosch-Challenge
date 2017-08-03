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
                               const VehicleMap& vehicles);

  Vehicle::Trajectory Generate(const Vehicle::State& begin_s,
                               const Vehicle::State& begin_d,
                               std::size_t target_vehicle_id,
                               const Vehicle::State& delta_s,
                               const Vehicle::State& delta_d,
                               double target_time,
                               const VehicleMap& vehicles);

private:
  std::random_device random_device_;
  std::default_random_engine rng_;
  std::normal_distribution<double> dist_s_;
  std::normal_distribution<double> dist_s_dot_;
  std::normal_distribution<double> dist_s_double_dot_;
  std::normal_distribution<double> dist_d_;
  std::normal_distribution<double> dist_d_dot_;
  std::normal_distribution<double> dist_d_double_dot_;

  Vehicle::State PerturbS(const Vehicle::State& s);
  Vehicle::State PerturbD(const Vehicle::State& d);
};

#endif // TRAJECTORYGENERATOR_H
