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
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    const string map_file_ = MAP_FILE_PATH;

    // Parse the map file and fill up the map waypoints
    ParseMapFile(map_file_, 
                 map_waypoints_x, 
                 map_waypoints_y,
                 map_waypoints_s,
                 map_waypoints_dx,
                 map_waypoints_dy);


    // EgoVechile start with middle lane
    int ego_lane = 1;

    // Reference velocity.
    double ref_vel = 0.0;  // mph

    h.onMessage([&ev,
                 &ref_vel, 
                 &ego_lane, 
                 &map_waypoints_x, 
                 &map_waypoints_y, 
                 &map_waypoints_s,
                 &map_waypoints_dx, 
                 &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, 
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
            
            // Provided previous path point size.
            const int prev_size = previous_path_x.size();

            // Preventing collitions.
            car_s = prev_size ? end_path_s : car_s;

            // Prediction : Analysing other cars positions.
            bool car_ahead = false;
            bool car_left  = false;
            bool car_righ  = false;

        
            
            for (int i = 0; i < sensor_fusion.size(); i++) {
                // Find car speed.
                const double vx    = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_VX];
                const double vy    = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_VY];

                double check_car_s = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_S];
                const float frenet_d = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_D];

                const LaneNumber lane = MapFrenetDToLane(frenet_d);

                if (lane == LaneInvalid)  continue; 

                const double check_speed = sqrt(vx * vx + vy * vy);

                // Estimate car s position after executing previous trajectory.
                check_car_s += ((double)prev_size * PREDICTION_SUBDURATION * check_speed);

                if (lane == ego_lane) {
                    // Car in our lane.
                    car_ahead |= 
                      check_car_s > car_s && check_car_s - car_s < MIN_SAFE_DISTRANCE;
                } else if (lane - ego_lane == -1) {
                    // Car left
                    car_left |= 
                      car_s - MIN_SAFE_DISTRANCE < check_car_s && car_s + MIN_SAFE_DISTRANCE > check_car_s;
                } else if (lane - ego_lane == 1) {
                    // Car right
                    car_righ |= 
                      car_s - MIN_SAFE_DISTRANCE < check_car_s && car_s + MIN_SAFE_DISTRANCE > check_car_s;
                }
            }

            // Behavior : Let's see what to do.
            double speed_diff = 0;

            if (car_ahead) {  // Car ahead
                if (!car_left && ego_lane > 0) {
                    // if there is no car left and there is a left lane.
                    ego_lane--;  // Change lane left.
                } else if (!car_righ && ego_lane != 2) {
                    // if there is no car right and there is a right lane.
                    ego_lane++;  // Change lane right.
                } else {
                    // cannot switch lane, slow down
                    speed_diff -= MAX_ACC;
                }
            } else {
                if (ego_lane != 1) {  // if we are not on the center lane.
                    if ((ego_lane == 0 && !car_righ) || (ego_lane == 2 && !car_left)) {
                        ego_lane = 1;  // Back to center.
                    }
                }
                if (ref_vel < MAX_SPEED) { speed_diff += MAX_ACC; }
            }

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x   = car_x;
            double ref_y   = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // Do I have have previous points
            if (prev_size < 2) {
                // There are not too many...
                const double prev_car_x = car_x - cos(car_yaw);
                const double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);

            } else {
                // Use the last two points.
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                const double ref_x_prev = previous_path_x[prev_size - 2];
                const double ref_y_prev = previous_path_y[prev_size - 2];

                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // Setting up target points in the future.
            vector<double> next_wp0 = getXY(car_s + MIN_SAFE_DISTRANCE, 
                                            2 + 4 * ego_lane, map_waypoints_s,
                                            map_waypoints_x, map_waypoints_y);

            vector<double> next_wp1 = getXY(car_s + MIN_SAFE_DISTRANCE * 2, 
                                            2 + 4 * ego_lane, map_waypoints_s,
                                            map_waypoints_x, map_waypoints_y);

            vector<double> next_wp2 = getXY(car_s + MIN_SAFE_DISTRANCE * 3, 
                                            2 + 4 * ego_lane, map_waypoints_s,
                                            map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Making coordinates to local car coordinates.
            for (int i = 0; i < ptsx.size(); i++) {

                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create the spline.
            tk::spline spl;
            spl.set_points(ptsx, ptsy);

            // Output path points from previous path for continuity.
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (int i = 0; i < prev_size; i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate distance y position on 30m ahead.
            double target_x    = MIN_SAFE_DISTRANCE;
            double target_y    = spl(target_x);
            double target_dist = sqrt(target_x * target_x + target_y * target_y);

            double x_add_on = 0;

            for (int i = 1; i < PREDICTION_INTERVALS - prev_size; i++) {

                ref_vel += speed_diff;
                ref_vel = min(ref_vel, MAX_SPEED);
                ref_vel = max(ref_vel, MAX_ACC);

                const double N = target_dist / (PREDICTION_SUBDURATION * ref_vel / 2.24);

                double x_point = x_add_on + target_x / N;
                double y_point = spl(x_point);

                x_add_on = x_point;

                const double x_ref = x_point;
                const double y_ref = y_point;

                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            } // for (int i = 1; i < 50 - prev_size; i++)

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