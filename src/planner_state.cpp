#include "planner_state.h"
#include <cassert>
#include <iostream>

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

// Local Helper-Functions
// -----------------------------------------------------------------------------

std::size_t GetLaneNumber(double d, double lane_width) {
  return d / lane_width;
}

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
              << ", s " << vehicle_s[0] << ", s_dot " << vehicle_s[1] << ", s_double_dot " << vehicle_s[2]
              << ", d " << vehicle_d[0] << ", d_dot " << vehicle_d[1] << ", d_double_dot " << vehicle_d[2]
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

  return adjacent_vehicles;
}

double GetLaneNumberCost(std::size_t lane) {
  return 1. / (lane + 1);
}

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
      || vehicle_ahead->second.s > 4. * preferred_buffer
      || vehicle_ahead->second.s_dot > preferred_speed) {
    return 0;
  }

  return (preferred_speed - vehicle_ahead->second.s_dot) / preferred_speed;
}

double GetLaneCost(Lane lane,
                   std::size_t target_lane,
                   double preferred_buffer,
                   double preferred_speed,
                   const AdjacentVehicles& adjacent_vehicles) {
  auto lane_nr_cost = GetLaneNumberCost(GetLaneNumber(lane, target_lane));
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

inline int GetTargetVehicleId(AdjacentVehicles::const_iterator vehicle_ahead,
                              const AdjacentVehicles& adjacent_vehicles,
                              double next_s,
                              double preferred_buffer) {
  if (vehicle_ahead != adjacent_vehicles.end()
      && vehicle_ahead->second.s < next_s + 2. * preferred_buffer) {
    return vehicle_ahead->second.id;
  }
  return -1;
}

bool IsLaneChangeSafe(AdjacentVehicles::const_iterator vehicle_ahead,
                      AdjacentVehicles::const_iterator vehicle_behind,
                      const AdjacentVehicles& adjacent_vehicles,
                      double preferred_buffer) {
  return (vehicle_ahead == adjacent_vehicles.end()
          || vehicle_ahead->second.s > preferred_buffer / 2.)
      && (vehicle_behind == adjacent_vehicles.end()
          || vehicle_behind->second.s < -preferred_buffer / 4.);
}

// PlannerState
// -----------------------------------------------------------------------------

PlannerState::PlannerState(std::size_t target_lane)
  : target_lane_(target_lane),
    target_vehicle_id_(-1) {
  // Empty.
}

void PlannerState::GetTarget(int& target_vehicle_id,
                             std::size_t& target_lane) {
  target_vehicle_id = target_vehicle_id_;
  target_lane = target_lane_;
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
  double preferred_buffer,
  double preferred_speed,
  double next_t,
  double next_s,
  double next_d,
  const VehicleMap& other_vehicles) {
  std::cout << "PlannerStateKeepingLane: next_s " << next_s
            << ", next_d " << next_d << std::endl;

  auto is_left_lane_available = target_lane_ > 0;
  auto is_right_lane_available = target_lane_ + 1 < n_lanes;

  auto adjacent_vehicles = GetAdjacentVehicles(target_lane_, n_lanes,
                                               lane_width, next_t, next_s,
                                               other_vehicles);
  for (const auto& v : adjacent_vehicles) {
    std::cout << "Adjacent vehicle type " << static_cast<int>(v.first)
              << ", id " << v.second.id << ", s " << v.second.s
              << ", s_dot " << v.second.s_dot << std::endl;
  }

  auto target_lane_cost = GetLaneCost(Lane::kTarget,
                                      target_lane_,
                                      preferred_buffer,
                                      preferred_speed,
                                      adjacent_vehicles);
  auto vehicle_ahead = adjacent_vehicles.find(AdjacentVehicleType::kAhead);

  auto left_lane_cost = 0.;
  auto right_lane_cost = 0.;
  auto vehicle_ahead_left = adjacent_vehicles.end();
  auto vehicle_ahead_right = adjacent_vehicles.end();
  auto vehicle_behind_left = adjacent_vehicles.end();
  auto vehicle_behind_right = adjacent_vehicles.end();
  if (is_left_lane_available) {
    left_lane_cost = GetLaneCost(Lane::kLeft,
                                 target_lane_,
                                 preferred_buffer,
                                 preferred_speed,
                                 adjacent_vehicles);
    vehicle_ahead_left = adjacent_vehicles.find(
      AdjacentVehicleType::kAheadLeft);
    vehicle_behind_left = adjacent_vehicles.find(
      AdjacentVehicleType::kBehindLeft);
  }
  if (is_right_lane_available) {
    right_lane_cost = GetLaneCost(Lane::kRight,
                                  target_lane_,
                                  preferred_buffer,
                                  preferred_speed,
                                  adjacent_vehicles);
    vehicle_ahead_right = adjacent_vehicles.find(
      AdjacentVehicleType::kAheadRight);
    vehicle_behind_right = adjacent_vehicles.find(
      AdjacentVehicleType::kBehindRight);
  }

  target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead,
                                          adjacent_vehicles,
                                          next_s,
                                          preferred_buffer);
  if (!is_left_lane_available) {
    // Check only target lane and right lane.
    if (right_lane_cost < target_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_right, vehicle_behind_right,
                           adjacent_vehicles, preferred_buffer)) {
        target_lane_ += 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_right,
                                                adjacent_vehicles,
                                                next_s,
                                                preferred_buffer);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneRight(*this));
      }
    }
  } else if (!is_right_lane_available) {
    // Check only target lane and left lane.
    if (left_lane_cost < target_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_left, vehicle_behind_left,
                           adjacent_vehicles, preferred_buffer)) {
        target_lane_ -= 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_left,
                                                adjacent_vehicles,
                                                next_s,
                                                preferred_buffer);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneLeft(*this));
      }
    }
  } else {
    // Check target, left, and right lanes.
    if (left_lane_cost < target_lane_cost && left_lane_cost < right_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_left, vehicle_behind_left,
                           adjacent_vehicles, preferred_buffer)) {
        target_lane_ -= 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_left,
                                                adjacent_vehicles,
                                                next_s,
                                                preferred_buffer);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneLeft(*this));
      }
    } else if (right_lane_cost < target_lane_cost
        && right_lane_cost < left_lane_cost) {
      if (IsLaneChangeSafe(vehicle_ahead_right, vehicle_behind_right,
                           adjacent_vehicles, preferred_buffer)) {
        target_lane_ += 1;
        target_vehicle_id_ = GetTargetVehicleId(vehicle_ahead_right,
                                                adjacent_vehicles,
                                                next_s,
                                                preferred_buffer);
        return std::shared_ptr<PlannerState>(
          new PlannerStateChangingLaneRight(*this));
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
  double preferred_buffer,
  double preferred_speed,
  double next_t,
  double next_s,
  double next_d,
  const VehicleMap& other_vehicles) {
  auto lane_number = GetLaneNumber(next_d, lane_width);
  std::cout << "PlannerStateChangingLaneLeft: next_s " << next_s
            << ", next_d " << next_d << ", lane_number " << lane_number
            << std::endl;
  if (lane_number == target_lane_) {
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
  double preferred_buffer,
  double preferred_speed,
  double next_t,
  double next_s,
  double next_d,
  const VehicleMap& other_vehicles) {
  auto lane_number = GetLaneNumber(next_d, lane_width);
  std::cout << "PlannerStateChangingLaneRight: next_s " << next_s
            << ", next_d " << next_d << ", lane_number " << lane_number
            << std::endl;
  if (lane_number == target_lane_) {
    return std::shared_ptr<PlannerState>(new PlannerStateKeepingLane(*this));
  }
  return std::shared_ptr<PlannerState>();
}
