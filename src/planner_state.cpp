#include "planner_state.h"
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

// Local Types
// -----------------------------------------------------------------------------

enum class AdjacentVehicleType {
  kAhead,
  kAheadLeft,
  kAheadRight,
  kBehindLeft,
  kBehindRight
};

struct AdjacentVehicleData {
  std::size_t id;
  double s;
  double s_dot;
};

typedef std::map<AdjacentVehicleType,
                 AdjacentVehicleData> AdjacentVehicles;

enum class Lane {
  kTarget,
  kLeft,
  kRight
};

// Local Constants
// -----------------------------------------------------------------------------

// Preferred buffer time when changing lanes [s].
auto kPreferredBufferTime = 2.;

// Laterral acceleration (d_dot) when driving straight [m/s/s] 
auto kStraightDDot = 0.3;

// Additional cost of passing in a right lane.
auto kRightLanePassingCost = 1e-3;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Provides lane number.
// @param[in] d           d-coordinate.
// @param[in] lane_width  Lane width [m]
// @return  Lane number.
std::size_t GetLaneNumber(double d, double lane_width) {
  return d / lane_width;
}

// Provides lane number.
// @param[in] lane         Relative lane position w.r.t. target lane.
// @param[in] target_lane  Target lane number.
// @return  Lane number.
std::size_t GetLaneNumber(Lane lane, std::size_t target_lane) {
  switch (lane) {
    case Lane::kTarget:
      return target_lane;
    case Lane::kLeft:
      assert(target_lane > 0);
      return target_lane - 1;
    case Lane::kRight:
      return target_lane + 1;
  }
}

// Provides adjacent vehicles.
// @param[in] target_lane     Target lane number.
// @param[in] n_lanes         Number of lanes on the road.
// @param[in] lane_width      Lane width [m]
// @param[in] next_t          Next time step.
// @param[in] next_s          Next s-coordinate.
// @param[in] other_vehicles  Other vehicles.
// @return  Adjacent vehicles.
AdjacentVehicles GetAdjacentVehicles(
  std::size_t target_lane,
  std::size_t n_lanes,
  double lane_width,
  double next_t,
  double next_s,
  const VehicleMap& other_vehicles) {
  AdjacentVehicles adjacent_vehicles;

  auto is_left_lane_available = target_lane > 0;
  auto is_right_lane_available = target_lane + 1 < n_lanes;

  auto is_vehicle_ahead_detected = false;
  std::size_t ahead_vehicle_id = 0;
  auto ahead_vehicle_s = 0.;
  auto ahead_vehicle_s_dot = 0.;

  auto is_vehicle_ahead_left_detected = false;
  std::size_t vehicle_ahead_left_id = 0;
  auto vehicle_ahead_left_s = 0.;
  auto vehicle_ahead_left_s_dot = 0.;

  auto is_vehicle_ahead_right_detected = false;
  std::size_t vehicle_ahead_right_id = 0;
  auto vehicle_ahead_right_s = 0.;
  auto vehicle_ahead_right_s_dot = 0.;

  auto is_vehicle_behind_left_detected = false;
  std::size_t vehicle_behind_left_id = 0;
  auto vehicle_behind_left_s = 0.;
  auto vehicle_behind_left_s_dot = 0.;

  auto is_vehicle_behind_right_detected = false;
  std::size_t vehicle_behind_right_id = 0;
  auto vehicle_behind_right_s = 0.;
  auto vehicle_behind_right_s_dot = 0.;

  for (const auto& v : other_vehicles) {
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    v.second.GetState(next_t, vehicle_s, vehicle_d);
    auto lane = GetLaneNumber(vehicle_d[0], lane_width);
    std::cout << "Vehicle Id " << v.first
              << ", s " << vehicle_s[0] << ", s_dot " << vehicle_s[1]
              << ", s_double_dot " << vehicle_s[2]
              << ", d " << vehicle_d[0] << ", d_dot " << vehicle_d[1]
              << ", d_double_dot " << vehicle_d[2]
              << ", lane " << lane << std::endl;
    if (vehicle_s[0] > next_s) {
      // The other vehicle is ahead.
      if (lane == target_lane) {
        // The other vehicle is in target lane.
        if (!is_vehicle_ahead_detected || vehicle_s[0] < ahead_vehicle_s) {
          is_vehicle_ahead_detected = true;
          ahead_vehicle_id = v.first;
          ahead_vehicle_s = vehicle_s[0];
          ahead_vehicle_s_dot = vehicle_s[1];
        }
      } else if (is_left_lane_available && lane == target_lane - 1) {
        // The other vehicle is in left lane.
        if (!is_vehicle_ahead_left_detected
            || vehicle_s[0] < vehicle_ahead_left_s) {
          is_vehicle_ahead_left_detected = true;
          vehicle_ahead_left_id = v.first;
          vehicle_ahead_left_s = vehicle_s[0];
          vehicle_ahead_left_s_dot = vehicle_s[1];
        }
      } else if (is_right_lane_available && lane == target_lane + 1) {
        // The other vehicle is in right lane.
        if (!is_vehicle_ahead_right_detected
            || vehicle_s[0] < vehicle_ahead_right_s) {
          is_vehicle_ahead_right_detected = true;
          vehicle_ahead_right_id = v.first;
          vehicle_ahead_right_s = vehicle_s[0];
          vehicle_ahead_right_s_dot = vehicle_s[1];
        }
      }
    } else {
      // The other vehicle is behind.
      if (is_left_lane_available && lane == target_lane - 1) {
        // The other vehicle is in left lane.
        if (!is_vehicle_behind_left_detected
            || vehicle_s[0] > vehicle_behind_left_s) {
          is_vehicle_behind_left_detected = true;
          vehicle_behind_left_id = v.first;
          vehicle_behind_left_s = vehicle_s[0];
          vehicle_behind_left_s_dot = vehicle_s[1];
        }
      } else if (is_right_lane_available && lane == target_lane + 1) {
        // The other vehicle is in right lane.
        if (!is_vehicle_behind_right_detected
            || vehicle_s[0] > vehicle_behind_right_s) {
          is_vehicle_behind_right_detected = true;
          vehicle_behind_right_id = v.first;
          vehicle_behind_right_s = vehicle_s[0];
          vehicle_behind_right_s_dot = vehicle_s[1];
        }
      }
    }
  }

  if (is_vehicle_ahead_detected) {
    adjacent_vehicles.insert(std::make_pair(
      AdjacentVehicleType::kAhead,
      AdjacentVehicleData({ahead_vehicle_id,
                           ahead_vehicle_s,
                           ahead_vehicle_s_dot})
    ));
  }

  if (is_vehicle_ahead_left_detected) {
    adjacent_vehicles.insert(std::make_pair(
      AdjacentVehicleType::kAheadLeft,
      AdjacentVehicleData({vehicle_ahead_left_id,
                           vehicle_ahead_left_s,
                           vehicle_ahead_left_s_dot})
    ));
  }

  if (is_vehicle_ahead_right_detected) {
    adjacent_vehicles.insert(std::make_pair(
      AdjacentVehicleType::kAheadRight,
      AdjacentVehicleData({vehicle_ahead_right_id,
                           vehicle_ahead_right_s,
                           vehicle_ahead_right_s_dot})
    ));
  }

  if (is_vehicle_behind_left_detected) {
    adjacent_vehicles.insert(std::make_pair(
      AdjacentVehicleType::kBehindLeft,
      AdjacentVehicleData({vehicle_behind_left_id,
                           vehicle_behind_left_s,
                           vehicle_behind_left_s_dot})
    ));
  }

  if (is_vehicle_behind_right_detected) {
    adjacent_vehicles.insert(std::make_pair(
      AdjacentVehicleType::kBehindRight,
      AdjacentVehicleData({vehicle_behind_right_id,
                           vehicle_behind_right_s,
                           vehicle_behind_right_s_dot})
    ));
  }

  for (const auto& v : adjacent_vehicles) {
    std::cout << "Adjacent vehicle type " << static_cast<int>(v.first)
              << ", id " << v.second.id << ", s " << v.second.s
              << ", s_dot " << v.second.s_dot << std::endl;
  }

  return adjacent_vehicles;
}

// Computes cost of lane number.
// @param[in] lane     Lane number.
// @param[in] n_lanes  Number of lanes on the road.
// @return  Cost.
double GetLaneNumberCost(std::size_t lane, std::size_t n_lanes) {
  auto middle_lane = n_lanes / 2;
  auto cost = std::fabs(static_cast<double>(lane) - middle_lane) / n_lanes;
  if (lane > middle_lane) {
    cost += kRightLanePassingCost;
  }
  return cost;
}

// Computes cost of lane speed.
// @param[in] lane              Relative lane position w.r.t. target lane.
// @param[in] preferred_buffer  Preferred buffer with other vehicles [m]
// @param[in] preferred_speed   Preferred own speed [m/s]
// @param[in] vehicles          Adjacent vehicles.
// @return  Cost.
double GetLaneSpeedCost(Lane lane,
                        double preferred_buffer,
                        double preferred_speed,
                        const AdjacentVehicles& vehicles) {
  AdjacentVehicles::const_iterator vehicle_ahead;
  switch (lane) {
    case Lane::kTarget:
      vehicle_ahead = vehicles.find(AdjacentVehicleType::kAhead);
      break;
    case Lane::kLeft:
      vehicle_ahead = vehicles.find(AdjacentVehicleType::kAheadLeft);
      break;
    case Lane::kRight:
      vehicle_ahead = vehicles.find(AdjacentVehicleType::kAheadRight);
      break;
  }

  if (vehicle_ahead == vehicles.end()
      || vehicle_ahead->second.s > 3. * preferred_buffer
      || vehicle_ahead->second.s_dot > preferred_speed) {
    return 0;
  }

  return (preferred_speed - vehicle_ahead->second.s_dot) / preferred_speed;
}

// Computes total lane cost.
// @param[in] lane              Relative lane position w.r.t. target lane.
// @param[in] target_lane       Target lane number.
// @param[in] n_lanes           Number of lanes on the road.
// @param[in] preferred_buffer  Preferred buffer with other vehicles [m]
// @param[in] preferred_speed   Preferred own speed [m/s]
// @param[in] vehicles          Adjacent vehicles.
// @return  Cost.
double GetLaneCost(Lane lane,
                   std::size_t target_lane,
                   std::size_t n_lanes,
                   double preferred_buffer,
                   double preferred_speed,
                   const AdjacentVehicles& adjacent_vehicles) {
  auto lane_nr_cost = GetLaneNumberCost(GetLaneNumber(lane, target_lane),
                                        n_lanes);
  auto lane_speed_cost = GetLaneSpeedCost(lane,
                                          preferred_buffer,
                                          preferred_speed,
                                          adjacent_vehicles);
  auto total_cost = lane_nr_cost + 10. * lane_speed_cost;
  std::cout << "Lane " << static_cast<int>(lane) << ": lane_nr_cost "
            << lane_nr_cost << ", lane_speed_cost " << lane_speed_cost
            << ", total " << total_cost << std::endl;
  return total_cost;
}

// Provides adjacent vehicle Id.
// @param[in] vehicles  Adjacent vehicles.
// @param[in] type      Adjacent vehicle type.
// @return  Vehicle Id if the vehicle is detected, -1 otherwise.
inline int GetAdjacentVehicleId(const AdjacentVehicles& vehicles,
                                AdjacentVehicleType type) {
  auto vehicle = vehicles.find(type);
  if (vehicle != vehicles.end()) {
    return vehicle->second.id;
  }
  return -1;
}

// Provides target vehicle Id.
// @param[in] vehicle_ahead_id  Identifier of the vehicle ahead.
// @param[in] other_vehicles    Other vehicles on the road.
// @param[in] preferred_buffer  Preferred buffer with other vehicles [m]
// @return  Vehicle Id if the vehicle is detected, -1 otherwise.
inline int GetTargetVehicleId(int vehicle_ahead_id,
                              const VehicleMap& other_vehicles,
                              double preferred_buffer) {
  if (vehicle_ahead_id >= 0) {
    auto vehicle_ahead = other_vehicles.find(vehicle_ahead_id);
    assert(vehicle_ahead != other_vehicles.end());
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    vehicle_ahead->second.GetState(0, vehicle_s, vehicle_d);
    if (vehicle_s[0] > 0 && vehicle_s[0] < 2. * preferred_buffer) {
      return vehicle_ahead_id;
    }
  }
  return -1;
}

// Computes if a lane change is safe.
// @param[in] vehicle_ahead_id   Identifier of the vehicle ahead.
// @param[in] vehicle_behind_id  Identifier of the vehicle behind.
// @param[in] other_vehicles     Other vehicles on the road.
// @param[in]  current_speed     Current vehicle speed.
// @return  True if the lane change is safe, false otherwise.
inline bool IsLaneChangeSafe(int vehicle_ahead_id,
                             int vehicle_behind_id,
                             const VehicleMap& other_vehicles,
                             double current_speed) {
  auto is_ahead_safe = false;
  if (vehicle_ahead_id < 0) {
    is_ahead_safe = true;
  } else {
    auto vehicle_ahead = other_vehicles.find(vehicle_ahead_id);
    assert(vehicle_ahead != other_vehicles.end());
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    vehicle_ahead->second.GetState(0, vehicle_s, vehicle_d);
    is_ahead_safe = vehicle_s[0] > 0
      && vehicle_s[0] > current_speed * kPreferredBufferTime;
  }

  if (!is_ahead_safe) {
    return false;
  }

  auto is_behind_safe = false;
  if (vehicle_behind_id < 0) {
    is_behind_safe = true;
  } else {
    auto vehicle_behind = other_vehicles.find(vehicle_behind_id);
    assert(vehicle_behind != other_vehicles.end());
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    vehicle_behind->second.GetState(0, vehicle_s, vehicle_d);
    is_behind_safe = vehicle_s[0] < 0
      && vehicle_s[0] < -vehicle_s[1] * kPreferredBufferTime;
  }

  return is_ahead_safe && is_behind_safe;
}

} // namespace

// PlannerState
// -----------------------------------------------------------------------------

PlannerState::PlannerState(std::size_t target_lane)
  : lane_width_(),
    n_lanes_(),
    target_lane_(target_lane),
    target_vehicle_id_(-1),
    preferred_buffer_() {
  // Empty.
}

void PlannerState::GetTarget(int& target_vehicle_id,
                             std::size_t& target_lane) {
  target_vehicle_id = target_vehicle_id_;
  target_lane = target_lane_;
}

void PlannerState::UpdateTargetVehicleId(double next_t,
                                         double next_s,
                                         const VehicleMap& other_vehicles) {
  auto adjacent_vehicles = GetAdjacentVehicles(target_lane_, n_lanes_,
                                               lane_width_, next_t, next_s,
                                               other_vehicles);
  auto vehicle_ahead_id = GetAdjacentVehicleId(
    adjacent_vehicles, AdjacentVehicleType::kAhead);
  target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_id,
                                          other_vehicles,
                                          preferred_buffer_);
  std::cout << "Vehicle ahead Id " << vehicle_ahead_id
            << ", Target vehicle Id " << target_vehicle_id_ << std::endl;
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
  std::size_t n_lanes,
  double lane_width,
  Vehicle::State current_s,
  Vehicle::State current_d,
  double preferred_speed,
  double next_t,
  double next_s,
  const VehicleMap& other_vehicles) {
  n_lanes_ = n_lanes;
  lane_width_ = lane_width;
  preferred_buffer_ = kPreferredBufferTime * preferred_speed;
  std::cout << "PlannerStateKeepingLane: current s ("
            << current_s[0] << "," << current_s[1] << "," << current_s[2]
            << "), current_d ("
            << current_d[0] << "," << current_d[1] << "," << current_d[2]
            << ")" << std::endl;

  auto is_left_lane_available = target_lane_ > 0;
  auto is_right_lane_available = target_lane_ + 1 < n_lanes;

  auto adjacent_vehicles = GetAdjacentVehicles(target_lane_, n_lanes,
                                               lane_width, next_t, next_s,
                                               other_vehicles);

  auto target_lane_cost = GetLaneCost(Lane::kTarget, target_lane_, n_lanes,
                                      preferred_buffer_, preferred_speed,
                                      adjacent_vehicles);
  auto left_lane_cost = 0.;
  auto right_lane_cost = 0.;

  auto vehicle_ahead_id = GetAdjacentVehicleId(
    adjacent_vehicles, AdjacentVehicleType::kAhead);
  auto vehicle_ahead_left_id = -1;
  auto vehicle_ahead_right_id = -1;
  auto vehicle_behind_left_id = -1;
  auto vehicle_behind_right_id = -1;
  if (is_left_lane_available) {
    left_lane_cost = GetLaneCost(Lane::kLeft, target_lane_, n_lanes,
                                 preferred_buffer_, preferred_speed,
                                 adjacent_vehicles);
    vehicle_ahead_left_id = GetAdjacentVehicleId(
      adjacent_vehicles, AdjacentVehicleType::kAheadLeft);
    vehicle_behind_left_id = GetAdjacentVehicleId(
      adjacent_vehicles, AdjacentVehicleType::kBehindLeft);
  }
  if (is_right_lane_available) {
    right_lane_cost = GetLaneCost(Lane::kRight, target_lane_, n_lanes,
                                  preferred_buffer_, preferred_speed,
                                  adjacent_vehicles);
    vehicle_ahead_right_id = GetAdjacentVehicleId(
      adjacent_vehicles, AdjacentVehicleType::kAheadRight);
    vehicle_behind_right_id = GetAdjacentVehicleId(
      adjacent_vehicles,  AdjacentVehicleType::kBehindRight);
  }

  target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_id,
                                          other_vehicles,
                                          preferred_buffer_);
  if (!is_left_lane_available) {
    // Check only target lane and right lane.
    if (right_lane_cost < target_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_right_id, vehicle_behind_right_id,
                           other_vehicles, current_s[1])) {
        target_lane_ += 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_right_id,
                                                other_vehicles,
                                                preferred_buffer_);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneRight(*this));
      } else {
        std::cout << "Changing lane right is unsafe" << std::endl;
      }
    }
  } else if (!is_right_lane_available) {
    // Check only target lane and left lane.
    if (left_lane_cost < target_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_left_id, vehicle_behind_left_id,
                           other_vehicles, current_s[1])) {
        target_lane_ -= 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_left_id,
                                                other_vehicles,
                                                preferred_buffer_);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneLeft(*this));
      } else {
        std::cout << "Changing lane left is unsafe" << std::endl;
      }
    }
  } else {
    // Check target, left, and right lanes.
    if (left_lane_cost < target_lane_cost && left_lane_cost < right_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_left_id, vehicle_behind_left_id,
                           other_vehicles, current_s[1])) {
        target_lane_ -= 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_left_id,
                                                other_vehicles,
                                                preferred_buffer_);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneLeft(*this));
      } else {
        std::cout << "Changing lane left is unsafe" << std::endl;
      }
    } else if (right_lane_cost < target_lane_cost
        && right_lane_cost < left_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_right_id, vehicle_behind_right_id,
                           other_vehicles, current_s[1])) {
        target_lane_ += 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_right_id,
                                                other_vehicles,
                                                preferred_buffer_);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneRight(*this));
      } else {
        std::cout << "Changing lane right is unsafe" << std::endl;
      }
    }
  }

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
  std::size_t n_lanes,
  double lane_width,
  Vehicle::State current_s,
  Vehicle::State current_d,
  double preferred_speed,
  double next_t,
  double /*next_s*/,
  const VehicleMap& other_vehicles) {
  auto lane_number = GetLaneNumber(current_d[0], lane_width);
  std::cout << "PlannerStateChangingLaneLeft: current_s ("
            << current_s[0] << "," << current_s[1] << "," << current_s[2]
            << ", current_d ("
            << current_d[0] << "," << current_d[1] << "," << current_d[2]
            << ")" << std::endl;
  if (lane_number == target_lane_ && std::fabs(current_d[1]) < kStraightDDot) {
    return std::shared_ptr<PlannerState>(new PlannerStateKeepingLane(*this));
  }
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
  std::size_t n_lanes,
  double lane_width,
  Vehicle::State current_s,
  Vehicle::State current_d,
  double preferred_speed,
  double next_t,
  double /*next_s*/,
  const VehicleMap& other_vehicles) {
  auto lane_number = GetLaneNumber(current_d[0], lane_width);
  std::cout << "PlannerStateChangingLaneRight: current_s ("
            << current_s[0] << "," << current_s[1] << "," << current_s[2]
            << ", current_d ("
            << current_d[0] << "," << current_d[1] << "," << current_d[2]
            << ")" << std::endl;
  if (lane_number == target_lane_ && std::fabs(current_d[1]) < kStraightDDot) {
    return std::shared_ptr<PlannerState>(new PlannerStateKeepingLane(*this));
  }
  return std::shared_ptr<PlannerState>();
}
