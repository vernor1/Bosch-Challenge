#ifndef TRAJECTORYESTIMATOR_H
#define TRAJECTORYESTIMATOR_H

#include <vector>
#include "vehicle.h"

class TrajectoryEstimator {
public:
  TrajectoryEstimator();

  double GetCost(const Vehicle::Trajectory& trajectory,
                 const Vehicle::State& target_s,
                 const Vehicle::State& target_d,
                 double target_time,
                 const VehicleMap& vehicles,
                 double d_limit,
                 double s_dot_limit,
                 bool is_verbose = false);

  // Penalizes trajectories that span a duration which is longer or shorter than
  // the duration requested.
  double GetTimeDiffCost(const Vehicle::Trajectory& trajectory,
                         const Vehicle::State& target_s,
                         const Vehicle::State& target_d,
                         double target_time,
                         const VehicleMap& vehicles,
                         double d_limit,
                         double s_dot_limit);

  // Penalizes trajectories whose s coordinate (and derivatives) differ from the
  // goal.
  double GetSdiffCost(const Vehicle::Trajectory& trajectory,
                      const Vehicle::State& target_s,
                      const Vehicle::State& target_d,
                      double target_time,
                      const VehicleMap& vehicles,
                      double d_limit,
                      double s_dot_limit);

  // Penalizes trajectories whose d coordinate (and derivatives) differ from the
  // goal.
  double GetDdiffCost(const Vehicle::Trajectory& trajectory,
                      const Vehicle::State& target_s,
                      const Vehicle::State& target_d,
                      double target_time,
                      const VehicleMap& vehicles,
                      double d_limit,
                      double s_dot_limit);

  // Binary cost function which penalizes collisions.
  double GetCollisionCost(const Vehicle::Trajectory& trajectory,
                          const Vehicle::State& target_s,
                          const Vehicle::State& target_d,
                          double target_time,
                          const VehicleMap& vehicles,
                          double d_limit,
                          double s_dot_limit);

  // Penalizes getting close to other vehicles.
  double GetBufferCost(const Vehicle::Trajectory& trajectory,
                       const Vehicle::State& target_s,
                       const Vehicle::State& target_d,
                       double target_time,
                       const VehicleMap& vehicles,
                       double d_limit,
                       double s_dot_limit);

  double GetOffRoadCost(const Vehicle::Trajectory& trajectory,
                        const Vehicle::State& target_s,
                        const Vehicle::State& target_d,
                        double target_time,
                        const VehicleMap& vehicles,
                        double d_limit,
                        double s_dot_limit);

  double GetSpeedingCost(const Vehicle::Trajectory& trajectory,
                         const Vehicle::State& target_s,
                         const Vehicle::State& target_d,
                         double target_time,
                         const VehicleMap& vehicles,
                         double d_limit,
                         double s_dot_limit);

  // Rewards high average speeds.
  double GetEfficiencyCost(const Vehicle::Trajectory& trajectory,
                           const Vehicle::State& target_s,
                           const Vehicle::State& target_d,
                           double target_time,
                           const VehicleMap& vehicles,
                           double d_limit,
                           double s_dot_limit);

  double GetMaxAccelCost(const Vehicle::Trajectory& trajectory,
                         const Vehicle::State& target_s,
                         const Vehicle::State& target_d,
                         double target_time,
                         const VehicleMap& vehicles,
                         double d_limit,
                         double s_dot_limit);

  double GetTotalAccelCost(const Vehicle::Trajectory& trajectory,
                           const Vehicle::State& target_s,
                           const Vehicle::State& target_d,
                           double target_time,
                           const VehicleMap& vehicles,
                           double d_limit,
                           double s_dot_limit);

  double GetMaxJerkCost(const Vehicle::Trajectory& trajectory,
                        const Vehicle::State& target_s,
                        const Vehicle::State& target_d,
                        double target_time,
                        const VehicleMap& vehicles,
                        double d_limit,
                        double s_dot_limit);

  double GetTotalJerkCost(const Vehicle::Trajectory& trajectory,
                          const Vehicle::State& target_s,
                          const Vehicle::State& target_d,
                          double target_time,
                          const VehicleMap& vehicles,
                          double d_limit,
                          double s_dot_limit);

private:
  bool is_closest_distance_computed_;
  double closest_distance_;
};

#endif // TRAJECTORYESTIMATOR_H
