#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include <sstream>
#include <string>

#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
static std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

static ofstream s_ukf_logger("ukf.log");
static ofstream s_dataset("dataset1.txt");

static bool log_measurement = false;
static bool log_ukf = false;

static VectorXd process_measurement(UKF &ukf, Tools &tools,
                                    vector<VectorXd> &estimations, vector<VectorXd> &ground_truth, const string s)
{

  VectorXd RMSE = VectorXd(4);
  RMSE.fill(100.0);

  if (s == "")
    return RMSE;

  auto j = json::parse(s);
  std::string event = j[0].get<std::string>();

  if (event != "telemetry")
    return RMSE;
  // j[1] is the data JSON object
  string sensor_measurment = j[1]["sensor_measurement"];
  MeasurementPackage meas_package;
  istringstream iss(sensor_measurment);
  long long timestamp;

  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;

  if (sensor_type.compare("L") == 0)
  {
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    float px;
    float py;
    iss >> px;
    iss >> py;
    meas_package.raw_measurements_ << px, py;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  }
  else if (sensor_type.compare("R") == 0)
  {
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);
    float ro;
    float theta;
    float ro_dot;
    iss >> ro;
    iss >> theta;
    iss >> ro_dot;
    meas_package.raw_measurements_ << ro, theta, ro_dot;
    iss >> timestamp;
    meas_package.timestamp_ = timestamp;
  }

  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;
  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  VectorXd gt_values(4);
  gt_values(0) = x_gt;
  gt_values(1) = y_gt;
  gt_values(2) = vx_gt;
  gt_values(3) = vy_gt;
  ground_truth.push_back(gt_values);

  //Call ProcessMeasurment(meas_package) for Kalman filter
  double sensor_nis = ukf.ProcessMeasurement(meas_package);

  //Push the current estimated x,y positon from the Kalman filter's state vector

  VectorXd estimate(4);

  double p_x = ukf.x_(0);
  double p_y = ukf.x_(1);
  double v = ukf.x_(2);
  double yaw = ukf.x_(3);

  double v1 = cos(yaw) * v;
  double v2 = sin(yaw) * v;

  estimate(0) = p_x;
  estimate(1) = p_y;
  estimate(2) = v1;
  estimate(3) = v2;

  estimations.push_back(estimate);
  RMSE = tools.CalculateRMSE(estimations, ground_truth);

  if (log_measurement)
  {
    s_dataset << s;
    s_dataset << "\n";
    s_dataset.flush();
  }

  if (log_ukf)
  {
    // Log the NIS of the sensor
    if (ukf.UseLaser() && sensor_type.compare("L") == 0)
    {
      s_ukf_logger << "LaserNIS:";
      s_ukf_logger << sensor_nis;
    }
    else if (ukf.UseRadar() && sensor_type.compare("R") == 0)
    {
      s_ukf_logger << "RadarNIS:";
      s_ukf_logger << sensor_nis;
    }
    s_ukf_logger << "\n";
    // Log the estimation and ground truth
    s_ukf_logger << "Tracking:";
    for (int i = 0; i < 4; i++)
    {
      s_ukf_logger << estimate(i);
      s_ukf_logger << " ";
    }
    for (int i = 0; i < 4; i++)
    {
      s_ukf_logger << gt_values(i);
      s_ukf_logger << " ";
    }
    s_ukf_logger << "\n";
    s_ukf_logger.flush();
  } //  if (log_ukf)

  return RMSE;
}

static VectorXd run_ukf_dataset(std::string dataset, UKF &ukf)
{
  ifstream meas_data(dataset);
  string data;
  Tools tools;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  VectorXd RMSE;

  while (getline(meas_data, data))
  {
    auto s = hasData(std::string(data));
    if (s == "")
      continue;
    RMSE = process_measurement(ukf, tools, estimations, ground_truth, s);
  }
  return RMSE;
}

static void grid_search_stda_stdyaw()
{
  double std_a_min = 0.05;
  double std_a_max = 3;

  double std_yawdd_min = 0.05;
  double std_yawdd_max = 3;

  double min_error1 = 100;
  double min_error2 = 100;
  double delta = 0.05;

  for (double std_a = std_a_min; std_a < std_a_max; std_a += delta)
  {
    cout << "std_a= " << std_a << endl;
    for (double std_yawdd = std_yawdd_min; std_yawdd < std_yawdd_max; std_yawdd += 0.1)
    {
      UKF ukf1(std_a, std_yawdd);
      UKF ukf2(std_a, std_yawdd);

      VectorXd RMSE1 = run_ukf_dataset("../data/dataset1.txt", ukf1);
      VectorXd RMSE2 = run_ukf_dataset("../data/dataset2.txt", ukf2);

      bool good_for_dataset1 =
          (RMSE1(0) < 0.09) &&
          (RMSE1(1) < 0.1) &&
          (RMSE1(2) < 0.4) &&
          (RMSE1(3) < 0.3);

      bool good_for_dataset2 =
          (RMSE2(0) < 0.09) &&
          (RMSE2(1) < 0.1) &&
          (RMSE2(2) < 0.4) &&
          (RMSE2(3) < 0.3);

      double error1 = RMSE1.transpose() * RMSE1;
      double error2 = RMSE2.transpose() * RMSE2;

      if (good_for_dataset1)
      {
        if (error1 < min_error1)
        {
          cout << "****************" << endl;
          min_error1 = error1;
          cout << "std_a1: " << std_a << endl;
          cout << "std_yawdd1: " << std_yawdd << endl;
          cout << "RMSE1: " << RMSE1 << endl;
          cout << "RMSE2: " << RMSE2 << endl;
          cout << "min_error1: " << min_error1 << endl;
          cout << "****************" << endl;
        }
      }

      if (good_for_dataset2)
      {
        if (error2 < min_error2)
        {
          cout << "****************" << endl;
          min_error2 = error2;
          cout << "std_a2: " << std_a << endl;
          cout << "std_yawdd2: " << std_yawdd << endl;
          cout << "RMSE1: " << RMSE1 << endl;
          cout << "RMSE2: " << RMSE2 << endl;
          cout << "min_error2: " << min_error2 << endl;
          cout << "****************" << endl;
        }
      }

      if (good_for_dataset1 && good_for_dataset2)
      {
        cout << "######################################################" << endl;
        cout << "Matched: " << std_a << endl;
        cout << "std_a: " << std_a << endl;
        cout << "std_yawdd: " << std_yawdd << endl;
        cout << "RMSE1: " << RMSE1 << endl;
        cout << "RMSE2: " << RMSE2 << endl;
        cout << "######################################################" << endl;
      }
    }
  }

  cout << "Search is done" << endl;
}

int main()
{
  // Create a Kalman Filter instance
  grid_search_stda_stdyaw();
  return 0;

  UKF ukf(1, 1);
  // used to compute the RMSE later
  uWS::Hub h;
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&ukf, &tools, &estimations, &ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          VectorXd RMSE = process_measurement(ukf, tools, estimations, ground_truth, s);
          json msgJson;
          msgJson["estimate_x"] = ukf.x_(0); //px
          msgJson["estimate_y"] = ukf.x_(1); //py
          msgJson["rmse_x"] = RMSE(0);
          msgJson["rmse_y"] = RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);

          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {

        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
