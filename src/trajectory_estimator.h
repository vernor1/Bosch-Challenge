#ifndef TRAJECTORYESTIMATOR_H
#define TRAJECTORYESTIMATOR_H

#include <functional>
#include <vector>
#include "vehicle.h"

// Estimates a given trajectory, provides its cost.
class TrajectoryEstimator {
public:
  // Constructor.
  TrajectoryEstimator();
  TrajectoryEstimator(const TrajectoryEstimator&) = delete;
  TrajectoryEstimator& operator=(const TrajectoryEstimator&) = delete;

  // Provides the cost of a trajectory.
  // @param[in] trajectory   Trajectory.
  // @param[in] target_s     Target state of s-coordinate.
  // @param[in] target_d     Target state of d-coordinate.
  // @param[in] target_time  Targter time.
  // @param[in] vehicles     Other vehicles.
  // @param[in] d_limit      Limit of d-coordinate (road width).
  // @param[in] s_dot_limit  Limit of s_dot (speed limit).
  // @param[in] is_verbose   Indicates if verbose output is needed.
  // @return  Cost of the trajectory.
  double GetCost(const Vehicle::Trajectory& trajectory,
                 const Vehicle::State& target_s,
                 const Vehicle::State& target_d,
                 double target_time,
                 const VehicleMap& vehicles,
                 double d_limit,
                 double s_dot_limit,
                 bool is_verbose = false);

  // Provides the partial cost of a trajectory. This method is used for testing
  // only, since the caller must know the internal names of cost functions.
  // @param[in] costFunctionName  Name of a cost function.
  // @param[in] trajectory        Trajectory to compute the cost for.
  // @param[in] target_s          Target state of s-coordinate.
  // @param[in] target_d          Target state of d-coordinate.
  // @param[in] target_time       Targter time.
  // @param[in] vehicles          Other vehicles.
  // @param[in] d_limit           Limit of d-coordinate (road width).
  // @param[in] s_dot_limit       Limit of s_dot (speed limit).
  // @return  Partial cost of the trajectory.
  double GetCost(const std::string& costFunctionName,
                 const Vehicle::Trajectory& trajectory,
                 const Vehicle::State& target_s,
                 const Vehicle::State& target_d,
                 double target_time,
                 const VehicleMap& vehicles,
                 double d_limit,
                 double s_dot_limit);

private:
  // Prototype of a cost function.
  // @param[in] trajectory        Trajectory to compute the cost for.
  // @param[in] target_s          Target state of s-coordinate.
  // @param[in] target_d          Target state of d-coordinate.
  // @param[in] target_time       Targter time.
  // @param[in] vehicles          Other vehicles.
  // @param[in] d_limit           Limit of d-coordinate (road width).
  // @param[in] s_dot_limit       Limit of s_dot (speed limit).
  // @return  Cost of the trajectory.
  typedef std::function<double(TrajectoryEstimator&,
                               const Vehicle::Trajectory& trajectory,
                               const Vehicle::State& target_s,
                               const Vehicle::State& target_d,
                               double target_time,
                               const VehicleMap& vehicles,
                               double d_limit,
                               double s_dot_limit)> Function;

  struct WeightedCostFunction {
    const char* name;
    double weight;
    Function function;
  };

  static const std::vector<WeightedCostFunction> kWeightedCostFunctions;

  bool is_target_dt_computed_;
  double target_dt_;
  bool is_trajectory_t_computed_;
  std::vector<double> trajectory_t_;
  bool is_target_t_computed_;
  std::vector<double> target_t_;
  bool is_s_dot_coeffs_computed_;
  std::vector<double> s_dot_coeffs_;
  bool is_d_dot_coeffs_computed_;
  std::vector<double> d_dot_coeffs_;
  bool is_s_double_dot_coeffs_computed_;
  std::vector<double> s_double_dot_coeffs_;
  bool is_d_double_dot_coeffs_computed_;
  std::vector<double> d_double_dot_coeffs_;
  bool is_trajectory_s_computed_;
  std::vector<double> trajectory_s_;
  bool is_trajectory_d_computed_;
  std::vector<double> trajectory_d_;
  bool is_target_s_double_dot_computed_;
  std::vector<double> target_s_double_dot_;
  bool is_target_d_double_dot_computed_;
  std::vector<double> target_d_double_dot_;
  bool is_target_jerk_computed_;
  std::vector<double> target_jerk_;
  bool is_closest_distance_computed_;
  double closest_distance_;

  inline void ComputeTargetDt(double time);
  void ComputeTrajectoryT(double time);
  void ComputeTargetT(double time);
  void ComputeSDotCoeffs(const std::vector<double>& s_coeffs);
  void ComputeDDotCoeffs(const std::vector<double>& d_coeffs);
  void ComputeSDoubleDotCoeffs(const std::vector<double>& s_coeffs);
  void ComputeDDoubleDotCoeffs(const std::vector<double>& d_coeffs);
  void ComputeTrajectoryS(double time, const std::vector<double>& s_coeffs);
  void ComputeTrajectoryD(double time, const std::vector<double>& d_coeffs);
  void ComputeTargetSDoubleDot(double time, const std::vector<double>& s_coeffs);
  void ComputeTargetDDoubleDot(double time, const std::vector<double>& d_coeffs);
  void ComputeTargetJerk(double time, const std::vector<double>& s_coeffs);
  double GetClosestDistanceToAnyVehicle(const Vehicle::Trajectory& trajectory,
                                        const VehicleMap& vehicles);

  // Cost Functions
  // ---------------------------------------------------------------------------

  // Penalizes trajectories that span a duration which is longer or shorter than
  // the duration requested.
  double GetTimeDiffCost(const Vehicle::Trajectory& trajectory,
                         const Vehicle::State& target_s,
                         const Vehicle::State& target_d,
                         double target_time,
                         const VehicleMap& vehicles,
                         double d_limit,
                         double s_dot_limit);

  // Penalizes trajectories whose s-coordinate (and derivatives) differ from the
  // goal.
  double GetSdiffCost(const Vehicle::Trajectory& trajectory,
                      const Vehicle::State& target_s,
                      const Vehicle::State& target_d,
                      double target_time,
                      const VehicleMap& vehicles,
                      double d_limit,
                      double s_dot_limit);

  // Penalizes trajectories whose d-coordinate (and derivatives) differ from the
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

  // Penalizes getting off-road.
  double GetOffRoadCost(const Vehicle::Trajectory& trajectory,
                        const Vehicle::State& target_s,
                        const Vehicle::State& target_d,
                        double target_time,
                        const VehicleMap& vehicles,
                        double d_limit,
                        double s_dot_limit);

  // Penalizes exceeding the speed limit.
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

  // Penalizes maximum instant s-acceleration.
  double GetMaxSAccelCost(const Vehicle::Trajectory& trajectory,
                          const Vehicle::State& target_s,
                          const Vehicle::State& target_d,
                          double target_time,
                          const VehicleMap& vehicles,
                          double d_limit,
                          double s_dot_limit);

  // Penalizes maximum instant d-acceleration.
  double GetMaxDAccelCost(const Vehicle::Trajectory& trajectory,
                          const Vehicle::State& target_s,
                          const Vehicle::State& target_d,
                          double target_time,
                          const VehicleMap& vehicles,
                          double d_limit,
                          double s_dot_limit);

  // Penalizes maximum sum of instant s- and d-acceleration.
  double GetMaxTotalAccelCost(const Vehicle::Trajectory& trajectory,
                              const Vehicle::State& target_s,
                              const Vehicle::State& target_d,
                              double target_time,
                              const VehicleMap& vehicles,
                              double d_limit,
                              double s_dot_limit);

  // Penalizes sum of s- and d-acceleration over time.
  double GetTotalAccelCost(const Vehicle::Trajectory& trajectory,
                           const Vehicle::State& target_s,
                           const Vehicle::State& target_d,
                           double target_time,
                           const VehicleMap& vehicles,
                           double d_limit,
                           double s_dot_limit);

  // Penalizes maximum instant s-jerk.
  double GetMaxJerkCost(const Vehicle::Trajectory& trajectory,
                        const Vehicle::State& target_s,
                        const Vehicle::State& target_d,
                        double target_time,
                        const VehicleMap& vehicles,
                        double d_limit,
                        double s_dot_limit);

  // Penalizes s-jerk over time.
  double GetTotalJerkCost(const Vehicle::Trajectory& trajectory,
                          const Vehicle::State& target_s,
                          const Vehicle::State& target_d,
                          double target_time,
                          const VehicleMap& vehicles,
                          double d_limit,
                          double s_dot_limit);
};

#endif // TRAJECTORYESTIMATOR_H
