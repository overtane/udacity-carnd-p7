
#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "UnscentedKalmanFilter.h"
#include "MeasurementPackage.h"
#include "GroundTruthPackage.h"
#include "Tools.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void usage() {
	cout << "UnscentedKF [-l|-r|-a <double>|-y <double>] input output" << endl;
	cout << "-l - include lidar measurements" << endl;
	cout << "-r - include radar measurements" << endl;
	cout << "(omitting -l and -r includes all measurements" << endl;
	cout << "-a <double> - stardard deviation of longitural acceleration noise" << endl;
	cout << "-y <double> - standard deviation of yaw acceleration noise" << endl;
	cout << "input - path to measurement input file" << endl;
	cout << "output - path to prediction output file" << endl;
}

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  string line;

  bool lidar_data = true; 
  bool radar_data = true;
  int n_meas = 0;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long timestamp;

    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // laser measurement
      if (!lidar_data)
          continue;

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;

      iss >> px;
      iss >> py;

      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // radar measurement
      if (!radar_data)
          continue;
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float rho;
      float theta;
      float rho_dot;

      iss >> rho;
      iss >> theta;
      iss >> rho_dot;

      meas_package.raw_measurements_ << rho, theta, rho_dot;

      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);

    n_meas++;
    //if (n_meas >= 94) break;
  }

  // Create a UKF instance
  UnscentedKalmanFilter ukf(5,.75);

  size_t number_of_measurements = measurement_pack_list.size();

  std::cout << number_of_measurements << std::endl;

  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)
  for (size_t k = 0; k < number_of_measurements; ++k) {
    // Call the UKF-based fusion
    std::cout << k << std::endl;
    double nis = ukf.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << nis << "\t"; // nis
    out_file_ << ukf.x_(0) << "\t"; // pos1 - est
    out_file_ << ukf.x_(1) << "\t"; // pos2 - est
    out_file_ << ukf.x_(2) * cos(ukf.x_(3)) << "\t"; // vel1
    out_file_ << ukf.x_(2) * sin(ukf.x_(3)) << "\t"; // vel2
    //out_file_ << ukf.x_(2) << "\t"; // vel_abs -est
    //out_file_ << ukf.x_(3) << "\t"; // yaw_angle -est
    //out_file_ << ukf.x_(4) << "\t"; // yaw_rate -est

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // p1 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";

      // p2 - meas
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float rho = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << rho * cos(phi) << "\t"; // p1_meas
      out_file_ << rho * sin(phi) << "\t"; // p2_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";
  
    VectorXd x(4);
    x << ukf.x_(0), ukf.x_(1),ukf.x_(2)*cos(ukf.x_(3)), ukf.x_(2)*sin(ukf.x_(3)); // TODO: vx and vy
    estimations.push_back(x);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << Tools::CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
