#ifndef PLANNERSTATE_H
#define PLANNERSTATE_H

#include <map>
#include <memory>
#include "vehicle.h"

// The set of classes below implements finite state machine of Path Planner.

// Abstract base state.
// -----------------------------------------------------------------------------
class PlannerState {
public:
  // Constructor.
  // @param target_lane  Initial lane number.
  PlannerState(std::size_t target_lane);

  // Provides the next state, if the current state changes as a result of
  // analyzing given indicators. Additionally, updates the target vehicle Id.
  // @param[in] n_lanes          Number of lanes on the road.
  // @param[in] lane_width       Lane width.
  // @param[in] current_s        State vector of current s-coord.
  // @param[in] current_s        State vector of current d-coord.
  // @param[in] preferred_speed  Preferred speed [m/s]
  // @param[in] next_t           Next time step.
  // @param[in] next_s           Next s-coordinate.
  // @param[in] other_vehicles   Other vehicles on the road.
  // @return  Next state if the current state changes, NULL otherwise.
  virtual std::shared_ptr<PlannerState> GetState(
    std::size_t n_lanes,
    double lane_width,
    Vehicle::State current_s,
    Vehicle::State current_d,
    double preferred_speed,
    double next_t,
    double next_s,
    const VehicleMap& other_vehicles) = 0;

  // Provides the target info.
  // @param[out] target_vehicle_id  Target vehicle Id if present, -1 otherwise.
  // @param[out] target_lane        Target lane number.
  virtual void GetTarget(int& target_vehicle_id,
                         std::size_t& target_lane);

  // Updates the target vehicle Id.
  // @param[in] next_t           Next time step.
  // @param[in] next_s           Next s-coordinate.
  // @param[in] other_vehicles   Other vehicles on the road.
  virtual void UpdateTargetVehicleId(double next_t,
                                     double next_s,
                                     const VehicleMap& other_vehicles);

protected:
  double lane_width_;
  std::size_t n_lanes_;
  std::size_t target_lane_;
  int target_vehicle_id_;
  double preferred_buffer_;
};

// Keeping lane state.
// -----------------------------------------------------------------------------

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

// Changing lane left state.
// -----------------------------------------------------------------------------

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

// Changing lane right state.
// -----------------------------------------------------------------------------

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
