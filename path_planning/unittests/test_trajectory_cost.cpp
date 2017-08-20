#include "gtest/gtest.h"
#include "../src/trajectory_estimator.h"

namespace {

const auto TARGET_TIME = 15.;
const auto D_LIMIT = 12.;
const auto S_DOT_LIMIT = 50.;
enum {TARGET_VEHICLE_ID = 0};
const Vehicle TARGET_VEHICLE = {{0, 10, 0}, {2, 0, 0}};
const VehicleMap VEHICLES = {{TARGET_VEHICLE_ID, TARGET_VEHICLE}};
const Vehicle::State DELTA_S = {-40, 0, 0};
const Vehicle::State COLLISION_DELTA_S = {-5, 0, 0};
const Vehicle::State DELTA_D = {0, 0, 0};
const Vehicle::Trajectory BEST_TRAJECTORY = {
  {1.00000000e+01, 1.00000000e+01, 0.00000000e+00,
   -1.01770812e-01, 8.97977754e-03, -2.11288883e-04},
  {6.00000000e+00, 0.00000000e+00, 0.00000000e+00,
   -8.14166497e-03, 7.18382203e-04, -1.69031107e-05},
  17
};
const Vehicle::Trajectory COLLISION_TRAJECTORY = {
  {1.00000000e+01, 1.00000000e+01, 0.00000000e+00,
   -3.05312436e-02, 2.69393326e-03, -6.33866650e-05},
  {6.00000000e+00, 0.00000000e+00, 0.00000000e+00, -8.14166497e-03,
   7.18382203e-04, -1.69031107e-05},
  17
};

void GetTargetState(const Vehicle& target_vehicle,
                    double target_time,
                    const Vehicle::State& delta_s,
                    const Vehicle::State& delta_d,
                    Vehicle::State& target_s,
                    Vehicle::State& target_d) {
  target_s.clear();
  target_d.clear();
  Vehicle::State target_vehicle_s;
  Vehicle::State target_vehicle_d;
  target_vehicle.GetState(target_time, target_vehicle_s, target_vehicle_d);
  std::transform(target_vehicle_s.begin(), target_vehicle_s.end(),
                 delta_s.begin(),
                 std::back_inserter(target_s), std::plus<double>());
  std::transform(target_vehicle_d.begin(), target_vehicle_d.end(),
                 delta_d.begin(),
                 std::back_inserter(target_d), std::plus<double>());
}

} // namespace

TEST(TrajectoryEstimator, TimeDiffCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_estimator.GetCost("TimeDiff", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("TimeDiff", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);
}

TEST(TrajectoryEstimator, SdiffCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("Sdiff", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(8.881784197e-14, cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("Sdiff", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, DdiffCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("Ddiff", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("Ddiff", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, EfficiencyCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("Efficiency", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.29833624069813736, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("Efficiency", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.030293758250092173, cost, 1e-3);
}

TEST(TrajectoryEstimator, MaxJerkCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("MaxJerk", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("MaxJerk", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);
}

TEST(TrajectoryEstimator, TotalJerkCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("TotalJerk", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.011853432018670507, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("TotalJerk", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0035561811532192067, cost, 1e-3);
}

TEST(TrajectoryEstimator, CollisionCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("Collision", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("Collision", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(100., cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, GetBufferCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("Buffer", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(4.38398629204, cost * 10., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("Buffer", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(8.21378211245, cost * 10., 1e-3);
}

TEST(TrajectoryEstimator, MaxTotalAccelCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("MaxTotalAccel", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 10, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("MaxTotalAccel", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 10, 1e-3);
}

TEST(TrajectoryEstimator, TotalAccelCost) {
  TrajectoryEstimator trajectory_estimator;
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator.GetCost("TotalAccel", BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.051463370631273486, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator.GetCost("TotalAccel", COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.018121509220275778, cost, 1e-3);
}
