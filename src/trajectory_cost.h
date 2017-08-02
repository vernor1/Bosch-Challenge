#ifndef TRAJECTORYCOST_H
#define TRAJECTORYCOST_H

#include <vector>
#include "vehicle.h"

namespace trajectory_cost {

typedef std::function<double(const Vehicle::Trajectory& trajectory,
                             std::size_t target_vehicle_id,
                             const Vehicle::State& delta_s,
                             const Vehicle::State& delta_d,
                             double target_time,
                             const VehicleMap& vehicles)> Function;

// Penalizes trajectories that span a duration which is longer or shorter than
// the duration requested.
double GetTimeDiffCost(const Vehicle::Trajectory& trajectory,
                       std::size_t target_vehicle_id,
                       const Vehicle::State& delta_s,
                       const Vehicle::State& delta_d,
                       double target_time,
                       const VehicleMap& vehicles);

// Penalizes trajectories whose s coordinate (and derivatives) differ from the
// goal.
double GetSdiffCost(const Vehicle::Trajectory& trajectory,
                    std::size_t target_vehicle_id,
                    const Vehicle::State& delta_s,
                    const Vehicle::State& delta_d,
                    double target_time,
                    const VehicleMap& vehicles);

// Penalizes trajectories whose d coordinate (and derivatives) differ from the
// goal.
double GetDdiffCost(const Vehicle::Trajectory& trajectory,
                    std::size_t target_vehicle_id,
                    const Vehicle::State& delta_s,
                    const Vehicle::State& delta_d,
                    double target_time,
                    const VehicleMap& vehicles);

// Binary cost function which penalizes collisions.
double GetCollisionCost(const Vehicle::Trajectory& trajectory,
                        std::size_t target_vehicle_id,
                        const Vehicle::State& delta_s,
                        const Vehicle::State& delta_d,
                        double target_time,
                        const VehicleMap& vehicles);

// Penalizes getting close to other vehicles.
double GetBufferCost(const Vehicle::Trajectory& trajectory,
                     std::size_t target_vehicle_id,
                     const Vehicle::State& delta_s,
                     const Vehicle::State& delta_d,
                     double target_time,
                     const VehicleMap& vehicles);

double GetOffRoadCost(const Vehicle::Trajectory& trajectory,
                      std::size_t target_vehicle_id,
                      const Vehicle::State& delta_s,
                      const Vehicle::State& delta_d,
                      double target_time,
                      const VehicleMap& vehicles);

double GetSpeedingCost(const Vehicle::Trajectory& trajectory,
                       std::size_t target_vehicle_id,
                       const Vehicle::State& delta_s,
                       const Vehicle::State& delta_d,
                       double target_time,
                       const VehicleMap& vehicles);

// Rewards high average speeds.
double GetEfficiencyCost(const Vehicle::Trajectory& trajectory,
                         std::size_t target_vehicle_id,
                         const Vehicle::State& delta_s,
                         const Vehicle::State& delta_d,
                         double target_time,
                         const VehicleMap& vehicles);

double GetMaxAccelCost(const Vehicle::Trajectory& trajectory,
                       std::size_t target_vehicle_id,
                       const Vehicle::State& delta_s,
                       const Vehicle::State& delta_d,
                       double target_time,
                       const VehicleMap& vehicles);

double GetTotalAccelCost(const Vehicle::Trajectory& trajectory,
                         std::size_t target_vehicle_id,
                         const Vehicle::State& delta_s,
                         const Vehicle::State& delta_d,
                         double target_time,
                         const VehicleMap& vehicles);

double GetMaxJerkCost(const Vehicle::Trajectory& trajectory,
                      std::size_t target_vehicle_id,
                      const Vehicle::State& delta_s,
                      const Vehicle::State& delta_d,
                      double target_time,
                      const VehicleMap& vehicles);

double GetTotalJerkCost(const Vehicle::Trajectory& trajectory,
                        std::size_t target_vehicle_id,
                        const Vehicle::State& delta_s,
                        const Vehicle::State& delta_d,
                        double target_time,
                        const VehicleMap& vehicles);

} // trajectory_cost

#endif // TRAJECTORYCOST_H
