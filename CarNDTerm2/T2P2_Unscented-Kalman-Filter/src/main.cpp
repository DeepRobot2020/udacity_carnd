#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include <sstream>
#include <string>
#include <random>
#include <tuple>

#include "ukf.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

static ofstream s_ukf_logger("ukf.log");
static ofstream s_dataset("dataset1.txt");
// whether log all the measurement to a text file. Used for grid search 
static bool log_measurement_enabled = false;
// whether log useful UKF output like senor NIS, estimation and ground truth of (px, py, vx, vy)  
static bool log_ukf_enabled = false;
// Whether to perform gird serach to find the optimal std_a, std_yawdd and initial P
static bool grid_search_enabled = false;
// Whether to run with simulator 
static bool run_with_simulator = false;

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
/**
 * Prints the usage information for the program.
 *
 * @param program_name  The name of the application.
 */
static void usage(char *program_name)
{
  fprintf(
      stderr,
      "\n"
      "  Usage: %s [OPTIONS]\n"
      "    -l arg  Log UKF processing results (senor NIS, estimation and ground truth)\n"
      "    -m arg  Log measurements from a dataset:\n"
      "    -g arg  Run grid search to find optimal std_a and std_yawdd\n"
      "    -s arg  Run with simulator\n"
      "    -h      Print this help message.\n"
      "\n",
      program_name);
}

void process_args(int argc, char *argv[])
{
  int c;

  while ((c = getopt(argc, argv, "l:m:g:s:h")) != -1)
  {
    switch (c)
    {
    case 'l':
      // Set input stream path
      log_ukf_enabled = atoi(optarg) ? true : false;
      break;
    case 'm':
      log_measurement_enabled = atoi(optarg) ? true : false;
      break;
    case 'g':
      grid_search_enabled = atoi(optarg) ? true : false;
      break;
    case 's':
      run_with_simulator = atoi(optarg) ? true : false;
      break;
    case 'h':
      usage(*argv);
    default:
      break;
    }
  }
}

static VectorXd process_measurement(UKF &ukf, Tools &tools,
                                    vector<VectorXd> &estimations, 
                                    vector<VectorXd> &ground_truth, 
                                    const string &measurement)
{

  VectorXd RMSE = VectorXd(4);
  RMSE.fill(100.0);

  if (measurement == "")
    return RMSE;

  auto j = json::parse(measurement);
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

  // Log the measurement value 
  if (log_measurement_enabled)
  {
    s_dataset << measurement;
    s_dataset << "\n";
    s_dataset.flush();
  }
  // Log the UKF output value 
  if (log_ukf_enabled)
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

    s_ukf_logger << ukf.P_ << endl;
    for(int i =0; i < meas_package.raw_measurements_.size(); i++) {
      s_ukf_logger << meas_package.raw_measurements_(i);
      s_ukf_logger << " ";
    }
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
  meas_data.close();
  return RMSE;
}

float float_precision(const double x, const int decDigits) {
    stringstream ss;
    ss << fixed;
    ss.precision(decDigits); // set # places after decimal
    ss << x;
    return stof(ss.str());
  }
  
static std::tuple<bool, double, double, MatrixXd> grid_search_stda_stdyaw(double std_a_min, double std_a_max,
                                                                          double std_yawdd_min, double std_yawdd_max)
{
  double delta = 0.01;
  bool found_result = false;

  bool best_std_a = 0.0;
  bool best_std_yawdd = 0.0;
  MatrixXd best_P = MatrixXd::Identity(5, 5);

  double min_search_error1 = 1000.0;
  double min_search_error2 = 1000.0;

  for (double std_a = std_a_min; std_a <= std_a_max; std_a += delta)
  {
    cout << "std_a: "<< std_a << endl;
    for (double std_yawdd = std_yawdd_min; std_yawdd <= std_yawdd_max; std_yawdd += 0.1)
    {

      MatrixXd P = MatrixXd::Identity(5, 5);
      std::random_device rd;
      std::mt19937 gen(rd());
      for (int i = 0; i < 5; i++)
      {
        std::normal_distribution<float> d(0, 1);
        float sample = d(gen);
        sample = float_precision(sample, 3);
        P(i, i) = fabs(sample);
      }

      UKF ukf1(std_a, std_yawdd, P);
      UKF ukf2(std_a, std_yawdd, P);

      VectorXd RMSE1 = run_ukf_dataset("../data/dataset1.txt", ukf1);
      VectorXd RMSE2 = run_ukf_dataset("../data/dataset2.txt", ukf2);

      bool good_for_dataset1 =
          (RMSE1(0) <= 0.09) &&
          (RMSE1(1) <= 0.1) &&
          (RMSE1(2) <= 0.4) &&
          (RMSE1(3) <= 0.3);

      bool good_for_dataset2 =
          (RMSE2(0) <= 0.09) &&
          (RMSE2(1) <= 0.1) &&
          (RMSE2(2) <= 0.4) &&
          (RMSE2(3) <= 0.3);

      double err1 = RMSE1.transpose() * RMSE1;
      double err2 = RMSE2.transpose() * RMSE2;

      if (good_for_dataset1)
      {
        if(err1 > min_search_error1)
          continue;
        min_search_error1 = err1;
        best_std_a = std_a;
        best_std_yawdd = std_yawdd;
        best_P = P;

        cout << "######################################################" << endl;
        cout << "std_a1: " << std_a << endl;
        cout << "std_yawdd1: " << std_yawdd << endl;
        cout << "P: " << P << endl;

        cout << "RMSE1: " << RMSE1 << endl;
        cout << "RMSE2: " << RMSE2 << endl;
        cout << "min_search_error1: " << min_search_error1 << endl;
        cout << "######################################################" << endl;
      }

      if (good_for_dataset2)
      {
        if(err2 > min_search_error2)
          continue;
        min_search_error2 = err2;
        best_std_a = std_a;
        best_std_yawdd = std_yawdd;
        best_P = P;

        cout << "######################################################" << endl;
        cout << "std_a2: " << std_a << endl;
        cout << "std_yawdd2: " << std_yawdd << endl;
        cout << "P: " << P << endl;
        cout << "RMSE1: " << RMSE1 << endl;
        cout << "RMSE2: " << RMSE2 << endl;
        cout << "min_search_error2: " << min_search_error2 << endl;
        cout << "######################################################" << endl;
      }
    }
  }

  if (!found_result)
  {
    cout << "Search failed: no good std_a and std_yawdd" << endl;
    return std::make_tuple(false, 1.0, 1.0, MatrixXd::Identity(5, 5));
  }
  else
  {
    cout << "Search is sucessful" << endl;
    return std::make_tuple(true, best_std_a, best_std_yawdd, best_P);
  }
}
  /** Note:
   ** To run with simulator only: ./UnscentedKF -s 1
   ** To perform grid search only: ./UnscentedKF -g 1
        The two dataset from the simulator have be pre-recorded into data 
    Usage: ./UnscentedKF [OPTIONS]
      -l arg  Log UKF processing results (senor NIS, estimation and ground truth)
      -m arg  Log measurements from a dataset:
      -r arg  Run grid search to find optimal std_a and std_yawdd
      -s arg  Run with simulator
      -h      Print this help message.
   */
int main(int argc, char *argv[])
{
  process_args(argc, argv);
  cout << "UKF Mode:" << endl;
  cout << "\tLog Measurement: " << (log_measurement_enabled ? "True" : "False") << endl;
  cout << "\tLog UKF: " << (log_ukf_enabled ? "True" : "False") << endl;
  cout << "\tGrid Search: " << (grid_search_enabled ? "True" : "False") << endl;
  cout << "\tRun with Simulator: " << (run_with_simulator ? "True" : "False") << endl;

   /*****************************************************************************
   *  Initialization
   ****************************************************************************/ 
  /** Below is a set of optimal value (std_a, std_yawdd, P) I got from grid search
   */
  double ukf_std_a = 0.5;
  double ukf_std_yawdd = 0.7;
  MatrixXd ukf_P = MatrixXd::Identity(5, 5);
  ukf_P(0,0) = 1.2;
  ukf_P(1,1) = 1.98;
  ukf_P(2,2) = 0.9;
  ukf_P(3,3) = 2.09;
  ukf_P(4,4) = 0.6;

  if (grid_search_enabled)
  {
    const double std_a_min = 0.1;
    const double std_a_max = 3;
    const double std_yawdd_min = 0.1;
    const double std_yawdd_max = 3;

    cout << "Runing grid search to find optimal std_a and std_yawdd..." << endl;
    cout << "Search range:" << endl;
    cout << "std_a: [" << std_a_min << ", " << std_a_max << "]" << endl;
    cout << "std_yawdd: [" << std_yawdd_min << ", " << std_yawdd_max << "]" << endl;
    // Create a Kalman Filter instance
    auto search_result = grid_search_stda_stdyaw(std_a_min, std_a_max, std_yawdd_min, std_yawdd_max);
    if (!run_with_simulator)
    {
      cout << "Search done, exiting:" << endl;
      return 0;
    }
    else
    {
      bool found_optimal = std::get<0>(search_result);
      if (found_optimal)
      {
        cout << "Updating optimal std_a, std_yawdd, P..." << endl;
        ukf_std_a     = std::get<1>(search_result);
        ukf_std_yawdd = std::get<2>(search_result);
        ukf_P         = std::get<3>(search_result);
      }
      else
      {
        cout << "No optimal std_a, std_yawdd, P, using default values" << endl;
      }
    }
  }

  if(!run_with_simulator)
    return 0;

  cout << "Running with simulator with: " << endl;
  cout << "\tstd_a: " << ukf_std_a << endl;
  cout << "\tstd_yawdd: " << ukf_std_yawdd << endl;
  cout << "\tP: " << ukf_P << endl;
  UKF ukf(ukf_std_a, ukf_std_yawdd, ukf_P);
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
