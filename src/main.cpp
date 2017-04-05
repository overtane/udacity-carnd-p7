
#include "Eigen/Dense"
#include "Measurement.h"
#include "Sensor.h"
#include "MeasurementFactory.h"
#include "UnscentedKalmanFilter.h"
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

// LOCAL FUNCTIONS
static void parse_arguments(int argc, char* argv[]);
static void check_and_open_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name);

void usage() {
	cout << "UnscentedKF [-l|-r|-n <in>|-p <int>|-a <double>|-y <double>|-d|-D] input output" << endl;
	cout << "    -l - include lidar measurements" << endl;
	cout << "    -r - include radar measurements" << endl;
	cout << "    (omitting -l and -r includes all measurements)" << endl;
	cout << "    -p <int> - minimum rate predictions are run" << endl;
	cout << "    -a <double> - stardard deviation of longitudal acceleration noise" << endl;
	cout << "    -y <double> - standard deviation of yaw acceleration noise" << endl;
	cout << "    -d - add some debugging output (add more d's to get more output)" << endl;
	cout << "    -n <int> - number of measurements, use only <int> first measurements of the data" << endl;
	cout << "    -m use modified version of the UKF algorithm for state covariance matrix" << endl;
	cout << "    -M use modified version of the UKF algorithm also for measurement covariance matrix" << endl;
	cout << "    input - path to measurement input file" << endl;
	cout << "    output - path to prediction output file" << endl;
}

// CONSTANTS
//
// Sensor dependent measurement noise constants
//
// Laser
const double std_laspx = 0.15;  // standard deviation position1 in m
const double std_laspy = 0.15;  // standard deviation position2 in m;
// Radar
const double std_radr = 0.3;    // standard deviation radius in m
const double std_radphi = 0.03; // standard deviation angle in rad
const double std_radrd = 0.3;   // standard deviation radius change in m/s

// GLOBALS
bool use_lidar = false;      // use lidar data
bool use_radar = false;      // use radar data
double std_a = 2.5;          // process noise: std deviation of acceleration
double std_yawdd = 0.8;      // process noise: std deviation of yaw
string in_filename;          // input filename
string out_filename;         // output filename
int debug = 0;               // debug output level
long n_meas = -1;            // number of measurements to use, negative value == use all
int pred_rate = 20;          // prediction rate, 0 == use measurement interval
bool state_modified = false; // use of state covariance matrix  modifications
bool meas_modified = false;  // use of measurement covariance matrix modifications

int main(int argc, char* argv[]) {

  parse_arguments(argc, argv);
  ifstream in_file(in_filename.c_str(), ifstream::in);
  ofstream out_file(out_filename.c_str(), ofstream::out);
  check_and_open_files(in_file, in_filename, out_file, out_filename);

  // Initialize covariance matrices of sensors
  VectorXd radar_noise(3);
  radar_noise << std_radr, std_radphi, std_radrd;
  VectorXd lidar_noise(2);
  lidar_noise << std_laspy, std_laspx;

  // Create sensor instances
  Sensor *radar = new RadarSensor("R", radar_noise, meas_modified); 
  Sensor *lidar = new LidarSensor("L", lidar_noise, meas_modified); 

  // Create sensor collection
  //
  // If a sensor is flagged out, its not included in sensor collection
  // and all measurements of the sensor are discarded.
  SensorContainer sensors;
  std::cout << "Sensors: ";
  if (use_lidar) {
      sensors[lidar->name_] = lidar;
      std::cout << "Lidar ";
  }
  if (use_radar) {
      sensors[radar->name_] = radar;
      std::cout << "Radar";  
  }
  std::cout << std::endl;

  if (debug>1) {
      if (use_radar) cout << "Radar covariance:\n" << radar->GetR() << endl;  
      if (use_lidar) cout << "Lidar covariance:\n" << lidar->GetR() << endl;  
  }

  // Get measurement factory
  MeasurementFactory* mf = MeasurementFactory::GetInstance();

  MeasurementContainer measurements;
  string line;

  // Read input file and fill measurement container
  while (getline(in_file, line) && n_meas != 0) {
    Measurement *m;
    istringstream iss(line);

    // Create measurement from input line, and store it to container
    // measurement is only created, if corresponding sensor is present in sensor collection
    m = mf->CreateMeasurement(iss, sensors);
    if (m != 0) {
        measurements.push_back(m);
        n_meas--;
    }
  }
  std::cout << "Number of measurements: " << measurements.size() << std::endl;
  in_file.close();

  // Create a filter instance
  UnscentedKalmanFilter ukf(std_a, std_yawdd, pred_rate, state_modified, debug);

  // Run measurement through the filter and get estimations
  MeasurementContainer::iterator it;
  int i;
  for(it=measurements.begin(), i=0; it!=measurements.end(); it++, i++) {

    Measurement *m = *it;
    if (debug)
        std::cout << "--- START MEASUREMENT: " << i << " -------------------------------" << std::endl;

    ukf.ProcessMeasurement(m);

    if (ukf.restart_)
        std::cout << "Filter restarted at measurement " << i << std::endl;

    out_file << *m; // output estimate 
  }
  out_file.close();


  // Calculate accuracy and check consistency
  // - print out results
  cout << "Accuracy - RMSE:" << endl << Tools::CalculateRMSE(measurements) << endl;
  cout << "Consistency - percentage above Chi^2(0.050):" << endl << Tools::CheckConsistency(measurements, sensors) << endl;
  
  cout << "Done!" << endl;
  return 0;
}



void parse_arguments(int argc, char* argv[]) {
 
    int opt;

    while ((opt = getopt(argc, argv, "mMp:n:dDlra:y:")) != -1) {
        switch (opt) {
        case 'l':
            use_lidar = true;
            break;
        case 'r':
            use_radar = true;
            break;
        case 'm':
            state_modified = true;
            break;
        case 'M':
            meas_modified = true;
            break;
        case 'd':
            debug++;
            break;
        case 'p':
	    pred_rate = atoi(optarg);
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

void check_and_open_files(ifstream& in_file, string& in_name,
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


