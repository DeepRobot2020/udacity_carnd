#ifndef _UTILS_H_
#define _UTILS_H_

#include <math.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

#include "constants.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }

inline double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x,
                    const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x,
                 const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x,
                         const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, 
double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                     const std::vector<double> &maps_y);

// Map a Frenet coordinate d value to a lane number
LaneNumber MapFrenetDToLane(int d);



void ParseMapFile(const std::string map_file,
    std::vector<double> &map_waypoints_x,
    std::vector<double> &map_waypoints_y,
    std::vector<double> &map_waypoints_s,
    std::vector<double> &map_waypoints_dx,
    std::vector<double> &map_waypoints_dy);

#endif //_UTILS_H_