#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#include <string>

// Constants
constexpr double MAX_SPEED        = 49.5;
constexpr double MAX_ACC          = .224;
constexpr double MAX_S            = 6945.554;
constexpr int MAX_TRACKED_VEHICLE_A_LANE  = 64;

constexpr double JSON_INDEX_EVENT_TYPE   = 0;
constexpr double JSON_INDEX_PAYLOAD      = 1;

constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_ID = 0;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_X  = 1;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_Y  = 2;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_VX = 3;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_VY = 4;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_S  = 5;
constexpr double JSON_INDEX_SENSOR_FUSION_VEHICLE_D  = 6;

constexpr double PREDICTION_DURATION    = 1; // 1 sec
constexpr double PREDICTION_INTERVALS   = 50; // 50 waypoints
constexpr double PREDICTION_SUBDURATION = PREDICTION_DURATION / PREDICTION_INTERVALS;
constexpr double MIN_SAFE_DISTRANCE     = 30.0; // meter


const std::string MAP_FILE_PATH = "../data/highway_map.csv";

// Enums 
enum LaneNumber { LaneInvalid = -1, LaneLeft, LaneMiddle, LaneRight, LaneNumberTotal };


#endif // _CONSTANTS_H 
