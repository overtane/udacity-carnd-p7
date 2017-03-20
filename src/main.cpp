
#include "Eigen/Dense"
#include "Measurement.h"
#include "MeasurementFactory.h"
#include "UnscentedKalmanFilter.h"
#include "MeasurementPackage.h"
#include "Tools.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

static void usage();

static void parse_arguments(int argc, char* argv[]);
static void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name);

void usage() {
	cout << "UnscentedKF [-l|-r|-n <in>|-a <double>|-y <double>|-d|-D] input output" << endl;
	cout << "    -l - include lidar measurements" << endl;
	cout << "    -r - include radar measurements" << endl;
	cout << "    (omitting -l and -r includes all measurements)" << endl;
	cout << "    -a <double> - stardard deviation of longitural acceleration noise" << endl;
	cout << "    -y <double> - standard deviation of yaw acceleration noise" << endl;
	cout << "    -d - add some debugging output (add more d's to get more output" << endl;
	cout << "    -n <int> - number of measurements, use only <int> first measurements of the data" << endl;
	cout << "    input - path to measurement input file" << endl;
	cout << "    output - path to prediction output file" << endl;
}

bool use_lidar = false;
bool use_radar = false;
double std_a = 5.0;
double std_yawdd = 0.5; 
string in_filename;
string out_filename;
int debug = 0;
long n_meas = -1;

void parse_arguments(int argc, char* argv[]) {
 
    int opt;

    while ((opt = getopt(argc, argv, "n:dDlra:y:")) != -1) {
        switch (opt) {
        case 'l':
            use_lidar = true;
            break;
        case 'r':
            use_radar = true;
            break;
        case 'd':
            debug++;
            break;
        case 'n':
	    n_meas = atoi(optarg);
            break;
        case 'a':
	    std_a = atof(optarg);
            break;
        case 'y':
	    std_yawdd = atof(optarg);
            break;
        default: /* '?' */
	    usage();
            exit(EXIT_FAILURE);
        }
    }

    if (!use_lidar && !use_radar)
	    use_lidar = use_radar = true;

    if (debug) {
        cout << "lidar: "    << use_lidar 
             << " radar: "   << use_radar 
             << " std_a: "   << std_a
	     << " std_yawdd: " << std_yawdd 
	     << " measurements: " << n_meas
	     << " debug: " << debug
	     << endl;
    }

    if (optind+2 != argc) {
	usage();
        exit(EXIT_FAILURE);
    }

    in_filename  = argv[optind];
    out_filename = argv[optind+1];

    if (debug) {
         cout << "in file: "  << in_filename  << endl;
        cout << "out file: " << out_filename << endl;
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

  parse_arguments(argc, argv);
  ifstream in_file_(in_filename.c_str(), ifstream::in);
  ofstream out_file_(out_filename.c_str(), ofstream::out);
  check_files(in_file_, in_filename, out_file_, out_filename);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  // Initialize covariance matrices for different sensor types
  // Laser measurement noise standard deviation position1 in m
  double std_laspx = 0.15;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy = 0.15;

  // Radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.3;

  VectorXd r_radar(3);
  r_radar << std_radr, std_radphi, std_radrd;
  RadarMeasurement::SetR(r_radar);
  VectorXd r_lidar(2);
  r_lidar << std_laspy, std_laspx;
  LidarMeasurement::SetR(r_lidar);

  if (debug>1) {
      cout << "Radar covariance:\n" << RadarMeasurement::R_ << endl;  
      cout << "Lidar covariance:\n" << LidarMeasurement::R_ << endl;  
  }

  MeasurementFactory* mf = MeasurementFactory::GetInstance();

  vector<MeasurementPackage> measurement_pack_list;
  vector<Measurement*> measurements;
  vector<VectorXd> ground_truth;
  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line) && n_meas != 0) {
    Measurement *m;
    istringstream iss(line);

    m = mf->CreateMeasurement(iss);
    if (m != 0) {
        measurements.push_back(m);

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        VectorXd gt(4);
        gt  << x_gt, y_gt, vx_gt, vy_gt;
        ground_truth.push_back(gt);

        n_meas--;
    }
  }

  // Create a UKF instance
  UnscentedKalmanFilter ukf(std_a, std_yawdd, debug);

  size_t number_of_measurements = measurements.size();

  std::cout << "Number of measurements: " << number_of_measurements << std::endl;

  vector<VectorXd> estimations;

  // start filtering from the second frame (the speed is unknown in the first
  // frame)
  for (size_t k = 0; k < number_of_measurements; ++k) {

    // Call the UKF-based fusion
    Measurement *m = measurements[k];
    
    if (debug)
        std::cout << k << std::endl;
    
    double nis = ukf.ProcessMeasurement(m);

    //cout << m->measurements_.transpose() << endl;
    //cout << ukf.x_.transpose() << endl;

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
    out_file_ << *m;

    // output the ground truth packages
    out_file_ << ground_truth[k](0) << "\t";
    out_file_ << ground_truth[k](1) << "\t";
    out_file_ << ground_truth[k](2) << "\t";
    out_file_ << ground_truth[k](3) << "\t";
  
    VectorXd x(4);
    x << ukf.x_(0), ukf.x_(1),ukf.x_(2)*cos(ukf.x_(3)), ukf.x_(2)*sin(ukf.x_(3)); // TODO: vx and v
    estimations.push_back(x);
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
