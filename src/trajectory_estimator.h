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
  bool is_trajectory_dt_computed_;
  double trajectory_dt_;
  bool is_target_dt_computed_;
  double target_dt_;
  bool is_trajectory_t_computed_;
  std::vector<double> trajectory_t_;
  bool is_target_t_computed_;
  std::vector<double> target_t_;
  bool is_s_dot_coeffs_computed_;
  std::vector<double> s_dot_coeffs_;
  bool is_s_double_dot_coeffs_computed_;
  std::vector<double> s_double_dot_coeffs_;
  bool is_jerk_coeffs_computed_;
  std::vector<double> jerk_coeffs_;
  bool is_trajectory_s_computed_;
  std::vector<double> trajectory_s_;
  bool is_trajectory_d_computed_;
  std::vector<double> trajectory_d_;
  bool is_target_s_double_dot_computed_;
  std::vector<double> target_s_double_dot_;
  bool is_target_jerk_computed_;
  std::vector<double> target_jerk_;
  bool is_closest_distance_computed_;
  double closest_distance_;

  inline void ComputeTrajectoryDt(double time);
  inline void ComputeTargetDt(double time);
  inline void ComputeTrajectoryT(double time);
  inline void ComputeTargetT(double time);
  inline void ComputeSDotCoeffs(const std::vector<double>& s_coeffs);
  inline void ComputeSDoubleDotCoeffs(const std::vector<double>& s_coeffs);
  inline void ComputeJerkCoeffs(const std::vector<double>& s_coeffs);
  inline void ComputeTrajectoryS(double time, const std::vector<double>& s_coeffs);
  inline void ComputeTrajectoryD(double time, const std::vector<double>& d_coeffs);
  inline void ComputeTargetSDoubleDot(double time, const std::vector<double>& s_coeffs);
  inline void ComputeTargetJerk(double time, const std::vector<double>& s_coeffs);

  double GetClosestDistanceToAnyVehicle(const Vehicle::Trajectory& trajectory,
                                        const VehicleMap& vehicles);
};

#endif // TRAJECTORYESTIMATOR_H
