#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "constants.h"
#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include "ego_vehicle.h"
#include "traffic.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {

    uWS::Hub h;
    EgoVehicle ev;
    
    h.onMessage([&ev](uWS::WebSocket<uWS::SERVER> ws, 
                      char *data,
                      size_t length, 
                      uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event

        // auto sdata = string(data).substr(0, length);
        // cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            const string s = hasData(data);

            if (s == "") { // Manual driving
              std::string msg = "42[\"manual\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT); 
              return;
            }

            const auto j = json::parse(s);

            // Get the event type 
            const string event = j[JSON_INDEX_EVENT_TYPE].get<string>();

            // Ignore events which are not "telemetry"
            if (event != "telemetry") return;

            // Ego Vehicle's loc information
            double car_s           = j[JSON_INDEX_PAYLOAD]["s"];
            double car_d           = j[JSON_INDEX_PAYLOAD]["d"];

            const double car_x     = j[JSON_INDEX_PAYLOAD]["x"];
            const double car_y     = j[JSON_INDEX_PAYLOAD]["y"];

            const double car_yaw   = j[JSON_INDEX_PAYLOAD]["yaw"];
            const double car_speed = j[JSON_INDEX_PAYLOAD]["speed"];

            // Previous path data given to the Planner
            const auto previous_path_x = j[JSON_INDEX_PAYLOAD]["previous_path_x"];
            const auto previous_path_y = j[JSON_INDEX_PAYLOAD]["previous_path_y"];

            // Previous path's end s and d values
            const double end_path_s = j[JSON_INDEX_PAYLOAD]["end_path_s"];
            const double end_path_d = j[JSON_INDEX_PAYLOAD]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            const auto sensor_fusion = j[JSON_INDEX_PAYLOAD]["sensor_fusion"];
    


            ev.UpdateState(car_s, car_d, car_x, car_y, car_yaw, car_speed, end_path_s, end_path_d, previous_path_x, previous_path_y);
            ev.UpdateTrafficInfo(sensor_fusion);
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            ev.GenerateNextTrajectory(next_x_vals, next_y_vals);

            json msgJson;

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            // this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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