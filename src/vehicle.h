#ifndef VEHICLE_H
#define VEHICLE_H

#include <unordered_map>
#include <vector>

// Implements other vehicle. Defines useful types and constants.
class Vehicle {
public:
  // Type of state vector, e.g. [s, s_dot, s_double_dot].
  typedef std::vector<double> State;

  // Type of other vehicle's trajectory.
  struct Trajectory {
    std::vector<double> s_coeffs;
    std::vector<double> d_coeffs;
    double time;
  };

  // Constant defines the state order, e.g. [s, s_dot, s_double_dot].
  enum {kStateOrder = 3};

  // Constant defines the standard deviation of s-coordinate.
  static const std::vector<double> kSigmaS;

  // Constant defines the standard deviation of d-coordinate.
  static const std::vector<double> kSigmaD;

  // Constructor.
  // @param begin_s  Initial state vector of s-coordinate.
  // @param begin_d  Initial state vector of d-coordinate.
  Vehicle(const State& begin_s, const State& begin_d);

  // Predicts the state at a given time.
  // @param[in]  t  Time for computing predicted state.
  // @param[out] s  State vector of s-coordinate at time t.
  // @param[out] d  State vector of d-coordinate at time t.
  void GetState(double t, State& s, State& d) const;

private:
  State begin_s_;
  State begin_d_;
};

// Mapping type of other vehicle Id -> Vehicle.
typedef std::unordered_map<std::size_t, Vehicle> VehicleMap;

#endif // VEHICLE_H
