#ifndef PLANNERSTATE_H
#define PLANNERSTATE_H

#include <map>
#include <memory>
#include "vehicle.h"

class PlannerState {
public:
  PlannerState(std::size_t target_lane);

  virtual std::shared_ptr<PlannerState> GetState(
    std::size_t n_lanes,
    double lane_width,
    Vehicle::State current_s,
    Vehicle::State current_d,
    double preferred_speed,
    double next_t,
    double next_s,
    const VehicleMap& other_vehicles) = 0;

  virtual void GetTarget(int& target_vehicle_id,
                         std::size_t& target_lane);
protected:
  std::size_t target_lane_;
  int target_vehicle_id_;
};

class PlannerStateKeepingLane : public PlannerState {
public:
  PlannerStateKeepingLane(std::size_t target_lane);
  PlannerStateKeepingLane(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    std::size_t n_lanes,
    double lane_width,
    Vehicle::State current_s,
    Vehicle::State current_d,
    double preferred_speed,
    double next_t,
    double next_s,
    const VehicleMap& other_vehicles) final;
};

class PlannerStateChangingLaneLeft : public PlannerState {
public:
  PlannerStateChangingLaneLeft(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    std::size_t n_lanes,
    double lane_width,
    Vehicle::State current_s,
    Vehicle::State current_d,
    double preferred_speed,
    double next_t,
    double next_s,
    const VehicleMap& other_vehicles) final;
};

class PlannerStateChangingLaneRight : public PlannerState {
public:
  PlannerStateChangingLaneRight(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    std::size_t n_lanes,
    double lane_width,
    Vehicle::State current_s,
    Vehicle::State current_d,
    double preferred_speed,
    double next_t,
    double next_s,
    const VehicleMap& other_vehicles) final;
};

#endif // PLANNERSTATE_H
