#include "gtest/gtest.h"
#include "../src/trajectory_cost.h"

namespace {

enum {TARGET_VEHICLE_ID = 0};
const auto TARGET_TIME = 15.;
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

TEST(trajectory_cost, GetTimeDiffCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetTimeDiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetTimeDiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0665680765023, cost, 1e-3);
}

TEST(trajectory_cost, GetSdiffCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetSdiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(8.881784197e-14, cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetSdiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0., cost * 100., 1e-3);
}

TEST(trajectory_cost, GetDdiffCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetDdiffCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetDdiffCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(6.66133814775e-14, cost * 100., 1e-3);
}

TEST(trajectory_cost, GetEfficiencyCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetEfficiencyCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
//  EXPECT_NEAR(0.298336249978, cost, 1e-3);
  EXPECT_NEAR(0., cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetEfficiencyCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
//  EXPECT_NEAR(0.0302937582174, cost, 1e-3);
  EXPECT_NEAR(0., cost, 1e-3);
}

TEST(trajectory_cost, GetMaxJerkCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetMaxJerkCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetMaxJerkCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0, cost * 10., 1e-3);
}

TEST(trajectory_cost, GetTotalJerkCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetTotalJerkCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0532924045646, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetTotalJerkCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0160015167532, cost, 1e-3);
}

TEST(trajectory_cost, GetCollisionCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetCollisionCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0., cost * 100., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetCollisionCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(100., cost * 100., 1e-3);
}

TEST(trajectory_cost, GetBufferCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetBufferCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(4.38398629204, cost * 10., 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetBufferCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(8.21378211245, cost * 10., 1e-3);
}

TEST(trajectory_cost, GetMaxAccelCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetMaxAccelCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0., cost * 10, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetMaxAccelCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0., cost * 10, 1e-3);
}

TEST(trajectory_cost, GetTotalAccelCost) {
  Vehicle::State target_s;
  Vehicle::State target_d;
  GetTargetState(TARGET_VEHICLE, BEST_TRAJECTORY.time,
    DELTA_S, DELTA_D, target_s, target_d);
  auto cost =  trajectory_cost::GetTotalAccelCost(BEST_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.321979347364, cost, 1e-3);

  GetTargetState(TARGET_VEHICLE, COLLISION_TRAJECTORY.time,
    COLLISION_DELTA_S, DELTA_D, target_s, target_d);
  cost =  trajectory_cost::GetTotalAccelCost(COLLISION_TRAJECTORY,
    target_s, target_d, TARGET_TIME, VEHICLES);
  EXPECT_NEAR(0.0998225885096, cost, 1e-3);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
