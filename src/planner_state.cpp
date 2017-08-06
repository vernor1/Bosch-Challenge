#include "planner_state.h"

// PlannerState
// -----------------------------------------------------------------------------

PlannerState::PlannerState(std::size_t target_lane)
  : target_lane_(target_lane) {
  // Empty.
}

void PlannerState::GetTarget(std::size_t& target_vehicle_id,
                             std::size_t& target_lane) {
  // TODO: Implement.
}

// PlannerStateKeepingLane
// -----------------------------------------------------------------------------

PlannerStateKeepingLane::PlannerStateKeepingLane(std::size_t target_lane)
  : PlannerState(target_lane) {
  // Empty.
}

PlannerStateKeepingLane::PlannerStateKeepingLane(
  const PlannerState& planner_state)
  : PlannerState(planner_state) {
  // Empty.
}

std::shared_ptr<PlannerState> PlannerStateKeepingLane::GetState(
  double lane_width,
  std::size_t n_lanes,
  const Vehicle& this_vehicle,
  const VehicleMap& other_vehicles) {
  return std::shared_ptr<PlannerState>();
}

// PlannerStateChangingLaneLeft
// -----------------------------------------------------------------------------

PlannerStateChangingLaneLeft::PlannerStateChangingLaneLeft(
  const PlannerState& planner_state)
  : PlannerState(planner_state) {
  // Empty.
}

std::shared_ptr<PlannerState> PlannerStateChangingLaneLeft::GetState(
  double lane_width,
  std::size_t n_lanes,
  const Vehicle& this_vehicle,
  const VehicleMap& other_vehicles) {
  return std::shared_ptr<PlannerState>();
}

// PlannerStateChangingLaneRight
// -----------------------------------------------------------------------------

PlannerStateChangingLaneRight::PlannerStateChangingLaneRight(
  const PlannerState& planner_state)
  : PlannerState(planner_state) {
  // Empty.
}

std::shared_ptr<PlannerState> PlannerStateChangingLaneRight::GetState(
  double lane_width,
  std::size_t n_lanes,
  const Vehicle& this_vehicle,
  const VehicleMap& other_vehicles) {
  return std::shared_ptr<PlannerState>();
}
