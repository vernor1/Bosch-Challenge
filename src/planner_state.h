#ifndef PLANNERSTATE_H
#define PLANNERSTATE_H

#include <memory>
#include "vehicle.h"

class PlannerState {
public:
  PlannerState(std::size_t target_lane);

  virtual std::shared_ptr<PlannerState> GetState(
    double lane_width,
    std::size_t n_lanes,
    const Vehicle& this_vehicle,
    const VehicleMap& other_vehicles) = 0;

  virtual void GetTarget(std::size_t& target_vehicle_id,
                         std::size_t& target_lane);
protected:
  std::size_t target_lane_;
};

class PlannerStateKeepingLane : public PlannerState {
public:
  PlannerStateKeepingLane(std::size_t target_lane);
  PlannerStateKeepingLane(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    double lane_width,
    std::size_t n_lanes,
    const Vehicle& this_vehicle,
    const VehicleMap& other_vehicles) final;
};

class PlannerStateChangingLaneLeft : public PlannerState {
public:
  PlannerStateChangingLaneLeft(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    double lane_width,
    std::size_t n_lanes,
    const Vehicle& this_vehicle,
    const VehicleMap& other_vehicles) final;
};

class PlannerStateChangingLaneRight : public PlannerState {
public:
  PlannerStateChangingLaneRight(const PlannerState& planner_state);

  std::shared_ptr<PlannerState> GetState(
    double lane_width,
    std::size_t n_lanes,
    const Vehicle& this_vehicle,
    const VehicleMap& other_vehicles) final;
};

#endif // PLANNERSTATE_H
