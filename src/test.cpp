#include <sstream>
#include "Python.h"
#include "trajectory_generator.h"

namespace {

enum {TARGET_VEHICLE_ID = 0};

/*
const auto TARGET_TIME = 15.;
const Vehicle TARGET_VEHICLE = {{0, 10, 0}, {2, 0, 0}};
const Vehicle::State DELTA_S = {-40, 0, 0};
const Vehicle::State DELTA_D = {0, 0, 0};
const Vehicle::State BEGIN_S = {10, 10, 0};
const Vehicle::State BEGIN_D = {6, 0, 0};
*/
const auto TARGET_TIME = 2.;
const Vehicle TARGET_VEHICLE = {{0, 15, 0}, {2, 0, 0}};
const Vehicle::State BEGIN_S = {0, 15, 0};
const Vehicle::State TARGET_S = {50, 15, 0};
const Vehicle::State BEGIN_D = {6, 0, 0};
const Vehicle::State TARGET_D = {10, 0, 0};

const VehicleMap VEHICLES = {{TARGET_VEHICLE_ID, TARGET_VEHICLE}};

// Takes the coefficients of a polynomial and creates a function of time from
// them.
std::function<double(double t)> GetFunction(std::vector<double> coeffs) {
  // TODO: Replace with function evaluation.
  auto f = [coeffs](double t) {
    auto result = 0.;
    for (std::size_t i = 0; i < coeffs.size(); ++i) {
      result += coeffs.at(i) * std::pow(t, i);
    }
    return result;
  };
  return f;
}

void ShowTrajectory(std::vector<double> s_coeffs,
                    std::vector<double> d_coeffs,
                    double time,
                    const Vehicle& vehicle) {
  auto fs = GetFunction(s_coeffs);
  auto fd = GetFunction(d_coeffs);
  auto t = 0.;
  std::ostringstream x;
  x << "X = [";
  std::ostringstream y;
  y << "Y = [";
  std::ostringstream x2;
  x2 << "X2 = [";
  std::ostringstream y2;
  y2 << "Y2 = [";
  while (t <= time + 0.01) {
    x << fs(t);
    y << fd(t);
    Vehicle::State vehicle_s;
    Vehicle::State vehicle_d;
    vehicle.GetState(t, vehicle_s, vehicle_d);
    x2 << vehicle_s[0];
    y2 << vehicle_d[0];
    t += 0.25;
    if (t <= time + 0.01) {
      x << ",";
      y << ",";
      x2 << ",";
      y2 << ",";
    }
  }
  x << "]";
  y << "]";
  x2 << "]";
  y2 << "]";

  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import matplotlib");
  PyRun_SimpleString("matplotlib.use('TkAgg')");
  PyRun_SimpleString("from matplotlib import pyplot as plt");
  PyRun_SimpleString("print(sys.version)");
  PyRun_SimpleString("print(matplotlib.__version__)");
  PyRun_SimpleString("plt.figure(figsize=(12, 8))");
  PyRun_SimpleString(x.str().c_str());
  PyRun_SimpleString(y.str().c_str());
  PyRun_SimpleString(x2.str().c_str());
  PyRun_SimpleString(y2.str().c_str());
  PyRun_SimpleString("plt.scatter(X,Y,color=\"blue\")");
  PyRun_SimpleString("for i in range(len(X)):\n  plt.annotate(int(i*0.25), (X[i], Y[i]), xytext=(X[i], Y[i]+0.1), fontsize=6)");
  PyRun_SimpleString("plt.scatter(X2, Y2,color=\"red\")");
  PyRun_SimpleString("for i in range(len(X2)):\n  plt.annotate(int(i*0.25), (X2[i], Y2[i]), xytext=(X2[i]-0.5, Y2[i]+0.1), fontsize=6)");
  PyRun_SimpleString("plt.show()");
  Py_Exit(0);
}

} // namespace

int main() {
  TrajectoryGenerator tg;
/*
  auto trajectory = tg.Generate(BEGIN_S, BEGIN_D,
                                TARGET_VEHICLE_ID, DELTA_S, DELTA_D,
                                TARGET_TIME,
                                VEHICLES);
*/
  auto trajectory = tg.Generate(BEGIN_S, BEGIN_D,
                                TARGET_S, TARGET_D,
                                TARGET_TIME,
                                VEHICLES);
  ShowTrajectory(trajectory.s_coeffs,
                 trajectory.d_coeffs,
                 trajectory.time,
                 TARGET_VEHICLE);
  return 0;
}
