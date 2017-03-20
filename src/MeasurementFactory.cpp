#include "Eigen/Dense"
#include "MeasurementFactory.h"
#include "Measurement.h"
#include <sstream>

using Eigen::VectorXd;
using std::string;

// static initializer of the factory instance
MeasurementFactory *MeasurementFactory::instance_ = 0;

Measurement *MeasurementFactory::CreateMeasurement(std::istringstream &iss) {

    string sensor_type;
    long timestamp;
    VectorXd measurements;

    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      float px;
      float py;

      iss >> px;
      iss >> py;

      measurements = VectorXd(2);
      measurements << px, py;

    } else if (sensor_type.compare("R") == 0) {
      float rho;
      float theta;
      float rho_dot;

      iss >> rho;
      iss >> theta;
      iss >> rho_dot;

      measurements = VectorXd(3);
      measurements << rho, theta, rho_dot;
    }

    iss >> timestamp;

    return CreateMeasurement(sensor_type, timestamp, measurements);
}

Measurement *MeasurementFactory::CreateMeasurement(string sensor_type, long timestamp, VectorXd measurements) {

    Measurement *m = 0; 

    if (sensor_type.compare("L") == 0) {
        m = new LidarMeasurement(timestamp, measurements);
    } else if (sensor_type.compare("R") == 0) {
        m = new RadarMeasurement(timestamp, measurements);
    }

    return m;
}

MeasurementFactory *MeasurementFactory::GetInstance() {
    if (instance_ == 0)
	    instance_ = new MeasurementFactory();
    return instance_;
}

