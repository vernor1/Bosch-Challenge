#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "path_planner.h"

using namespace std::placeholders;

namespace {

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
// @param[in] ws           WebSocket object.
// @param[in] steering     Steering value in [-1..1].
// @param[in] throttle     Throttle value in [-1..1].
// @param[in] predicted_x  X-coordinates of predicted waypoints.
// @param[in] predicted_y  Y-coordinates of predicted waypoints.
// @param[in] reference_x  X-coordinates of reference waypoints.
// @param[in] reference_y  Y-coordinates of reference waypoints.
void ControlSimulator(uWS::WebSocket<uWS::SERVER>& ws,
                      const std::vector<double>& next_x,
                      const std::vector<double>& next_y) {
  nlohmann::json json_msg;
  json_msg["next_x"] = next_x;
  json_msg["next_y"] = next_y;
  auto msg = "42[\"control\"," + json_msg.dump() + "]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

} // namespace

// main
// -----------------------------------------------------------------------------

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // TODO: Move to a function.
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  PathPlanner path_planner(map_waypoints_x,
                           map_waypoints_y,
                           map_waypoints_dx,
                           map_waypoints_dy,
                           map_waypoints_s,
                           max_s);

  h.onMessage([&path_planner](uWS::WebSocket<uWS::SERVER> ws,
                              char* data,
                              std::size_t length,
                              uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 indicates a websocket message
    // The 2 indicates a websocket event
    auto sdata = std::string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      std::string s = GetJsonData(sdata);
      if (!s.empty()) {
        auto j = nlohmann::json::parse(s);
        auto event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
            double x = j[1]["x"];
            double y = j[1]["y"];
            double s = j[1]["s"];
            double d = j[1]["d"];
            double yaw = j[1]["yaw"];
            double speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            std::vector<PathPlanner::DetectedVehicle> sensor_fusion;
            for (const auto& v : j[1]["sensor_fusion"]) {
              assert(v.size() == 7);
              sensor_fusion.push_back({v[0], v[1], v[2], v[3], v[4], v[5], v[6]});
            }

            path_planner.Update(x, y, s, d, yaw, speed,
                                previous_path_x, previous_path_y,
                                end_path_s, end_path_d,
                                sensor_fusion,
                                std::bind(ControlSimulator, ws, _1, _2));
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































