#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

namespace carnd {
  // Read data from json
  void from_json(const json & j, car_t & car) {

    car.id = j[0].get<int>();
    car.x = j[1];
    car.y = j[2];
    car.vx = j[3];
    car.vy = j[4];
    car.s = j[5];
    car.d = j[6];
  }

  void from_json(const json & j, ego_t & ego) {
    // Main car's localization Data
    ego.x = j["x"];
    ego.y = j["y"];
    ego.s = j["s"];
    ego.d = j["d"];
    ego.yaw = deg2rad(j["yaw"]);
    ego.v = mph2mps(j["speed"]);
    // Previous path data given to the Planner
    ego.previous_path.x = j["previous_path_x"].get<vector<double>>();
    ego.previous_path.y = j["previous_path_y"].get<vector<double>>();
    // Previous path's end s and d values 
    ego.end_path.s = j["end_path_s"];
    ego.end_path.d = j["end_path_d"];
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    ego.cars = j["sensor_fusion"].get<vector<car_t>>();
  }
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Path planner
  carnd::PathPlanner planner;
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  planner.initialize(map_file_);


  h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          
          carnd::path_t next_path;
          
          // run the planner to get x,y next path point
          // sample time is 0.02s
          planner.run(j[1], next_path, 0.02);
          
        	
        	json msgJson;
        	msgJson["next_x"] = next_path.x;
        	msgJson["next_y"] = next_path.y;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
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
