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

// TODO: Consider making a shared helper.
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

/*
    (time_diff_cost,    1),
    (s_diff_cost,       100),
    (d_diff_cost,       100),
    (efficiency_cost,   1),
    (max_jerk_cost,     10),
    (total_jerk_cost,   1),
    (collision_cost,    100),
    (buffer_cost,       10),
    (max_accel_cost,    10),
    (total_accel_cost,  1)
*/

TEST(TrajectoryEstimator, GetTimeDiffCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(
    new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_estimator->GetTimeDiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetTimeDiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);
}

TEST(TrajectoryEstimator, GetSdiffCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetSdiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(8.881784197e-14, cost * 100., 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetSdiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, GetDdiffCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetDdiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetDdiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, GetEfficiencyCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetEfficiencyCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.29833624069813736, cost, 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetEfficiencyCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.030293758250092173, cost, 1e-3);
}

TEST(TrajectoryEstimator, GetMaxJerkCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetMaxJerkCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetMaxJerkCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);
}

TEST(TrajectoryEstimator, GetTotalJerkCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetTotalJerkCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0532924045646, cost, 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetTotalJerkCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0160015167532, cost, 1e-3);
}

TEST(TrajectoryEstimator, GetCollisionCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetCollisionCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 100., 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetCollisionCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(100., cost * 100., 1e-3);
}

TEST(TrajectoryEstimator, GetBufferCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetBufferCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(4.38398629204, cost * 10., 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetBufferCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(8.21378211245, cost * 10., 1e-3);
}

TEST(TrajectoryEstimator, GetMaxAccelCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetMaxAccelCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 10, 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetMaxAccelCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0., cost * 10, 1e-3);
}

TEST(TrajectoryEstimator, GetTotalAccelCost) {
  std::shared_ptr<TrajectoryEstimator> trajectory_estimator(new TrajectoryEstimator);
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost = trajectory_estimator->GetTotalAccelCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.321979347364, cost, 1e-3);

  trajectory_estimator.reset(new TrajectoryEstimator);
  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost = trajectory_estimator->GetTotalAccelCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES, D_LIMIT, S_DOT_LIMIT);
  EXPECT_NEAR(0.0998225885096, cost, 1e-3);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
