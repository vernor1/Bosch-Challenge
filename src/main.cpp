#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "uWS/uWS.h"
#include "helpers.h"
#include "path_planner.h"

using namespace std::placeholders;

namespace {

// Local Constants
// -----------------------------------------------------------------------------

// TCP port accepting incoming connections from simulator.
enum { kTcpPort = 4567 };

// Lane wifth in [m].
const auto kLaneWidth = 4.;

// Number of lanes.
enum {kNumberOfLanes = 3};

// Speed limit in [m/s].
const auto kSpeedLimit = 50. * helpers::kMphToMps;

// Local Helper-Functions
// -----------------------------------------------------------------------------

// Checks if the SocketIO event has JSON data.
// @param[in] s  Raw event string
// @return       If there is data the JSON object in string format will be
//               returned, else an empty string will be returned.
std::string GetJsonData(const std::string& s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  return found_null == std::string::npos
    && b1 != std::string::npos
    && b2 != std::string::npos ? s.substr(b1, b2 - b1 + 1) : std::string();
}

// Sends a control message to the simulator.
// @param[in] ws      WebSocket object.
// @param[in] next_x  x-coordinates of next waypoints.
// @param[in] next_y  y-coordinates of next waypoints.
void ControlSimulator(uWS::WebSocket<uWS::SERVER>& ws,
                      const std::vector<double>& next_x,
                      const std::vector<double>& next_y) {
  nlohmann::json json_msg;
  json_msg["next_x"] = next_x;
  json_msg["next_y"] = next_y;
  auto msg = "42[\"control\"," + json_msg.dump() + "]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

// Creates an instance of Path Planner.
// @return          A smart pointer to the Path Planner object.
std::shared_ptr<PathPlanner> CreatePathPlanner() {
  // Load up map values for waypoint's x,y,s.
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;

  // Waypoint map to read from.
  std::string map_file_ = "../data/highway_map_bosch1.csv";

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
    auto x = 0.;
    auto y = 0.;
    auto s = 0.;
    auto dx = 0.;
    auto dy = 0.;
    iss >> x >> y >> s >> dx >> dy;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
  }

  return std::shared_ptr<PathPlanner>(new PathPlanner(map_waypoints_x,
                                                      map_waypoints_y,
                                                      map_waypoints_s));
}

} // namespace

// main
// -----------------------------------------------------------------------------

int main() {
  uWS::Hub hub;

  auto path_planner = CreatePathPlanner();
  hub.onMessage([&path_planner](uWS::WebSocket<uWS::SERVER> ws,
                               char* data,
                               std::size_t length,
                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 indicates a websocket message.
    // The 2 indicates a websocket event.
    auto sdata = std::string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = GetJsonData(sdata);
      if (!s.empty()) {
        auto j = nlohmann::json::parse(s);
        auto event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object.
          // Main car's localization Data.
          double s = j[1]["s"];
          double d = j[1]["d"];
          double x = j[1]["x"];
          double y = j[1]["y"];
          // Previous path data given to the Planner.
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          std::vector<PathPlanner::DetectedVehicle> sensor_fusion;
          for (const auto& v : j[1]["sensor_fusion"]) {
            assert(v.size() == 7);
            sensor_fusion.push_back({v[0], v[1], v[2], v[3], v[4], v[5], v[6]});
          }

          path_planner->Update(s, d, x, y, previous_path_x, previous_path_y,
                               sensor_fusion, kLaneWidth, kNumberOfLanes,
                               kSpeedLimit,
                               std::bind(ControlSimulator, ws, _1, _2));
        }
      } else {
        // Manual driving.
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  hub.onConnection([&path_planner](uWS::WebSocket<uWS::SERVER> ws,
                                   uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    path_planner = CreatePathPlanner();
  });

  hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws,
                         int code,
                         char* message,
                         size_t length) {
    ws.close();
    std::cout << "Disconnected." << std::endl;
  });

  if (hub.listen(kTcpPort)) {
    std::cout << "Listening on port " << kTcpPort << std::endl;
  } else {
    std::cerr << "Failed to listen on port " << kTcpPort << std::endl;
    return -1;
  }

  hub.run();
}
