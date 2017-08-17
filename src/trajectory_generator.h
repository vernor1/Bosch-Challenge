#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include <random>
#include "trajectory_estimator.h"
#include "vehicle.h"

// Implements Trajectory Generator.
class TrajectoryGenerator {
public:
  // Constructor.
  TrajectoryGenerator();
  TrajectoryGenerator(const TrajectoryGenerator&) = delete;
  TrajectoryGenerator& operator=(const TrajectoryGenerator&) = delete;

  // Generates a free-running trajectory of a vehicle.
  // @param[in] begin_s      Initial state vector of s-coordinate.
  // @param[in] begin_d      Initial state vector of d-coordinate.
  // @param[in] target_s     Target state vector of s-coordinate.
  // @param[in] target_d     Target state vector of d-coordinate.
  // @param[in] target_time  Target time.
  // @param[in] vehicles     Other vehicle's map.
  // @param[in] d_limit      Road width.
  // @param[in] s_dot_limit  Speed limit.
  // @return  Vehicle's trajectory.
  Vehicle::Trajectory Generate(const Vehicle::State& begin_s,
                               const Vehicle::State& begin_d,
                               const Vehicle::State& target_s,
                               const Vehicle::State& target_d,
                               double target_time,
                               const VehicleMap& vehicles,
                               double d_limit,
                               double s_dot_limit) const;

  // Generates a trajectory of a vehicle following other vehicle.
  // @param[in] begin_s      Initial state vector of s-coordinate.
  // @param[in] begin_d      Initial state vector of d-coordinate.
  // @param[in] target_vehicle_id  Other vehicle Id.
  // @param[in] delta_s      Delta state vector w.r.t. other's s-coordinate.
  // @param[in] delta_d      Delta state vector w.r.t. other's d-coordinate.
  // @param[in] target_time  Target time.
  // @param[in] vehicles     Other vehicle's map.
  // @param[in] d_limit      Road width.
  // @param[in] s_dot_limit  Speed limit.
  // @return  Vehicle's trajectory.
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
  mutable TrajectoryEstimator trajectory_estimator_;

  Vehicle::State PerturbS(const Vehicle::State& s) const;
  Vehicle::State PerturbD(const Vehicle::State& d) const;
};

#endif // TRAJECTORYGENERATOR_H
