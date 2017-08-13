#ifndef VEHICLE_H
#define VEHICLE_H

#include <unordered_map>
#include <vector>

class Vehicle {
public:
  typedef std::vector<double> State;
  struct Trajectory {
    std::vector<double> s_coeffs;
    std::vector<double> d_coeffs;
    double time;
  };
  enum {kStateOrder = 3};
  static const std::vector<double> kSigmaS;
  static const std::vector<double> kSigmaD;

  Vehicle(const State& begin_s, const State& begin_d);

  void GetState(double t, State& s, State& d) const;

private:
  State begin_s_;
  State begin_d_;
};

typedef std::unordered_map<std::size_t, Vehicle> VehicleMap;

#endif // VEHICLE_H
