#include "Eigen/Dense"
#include "MeasurementFactory.h"
#include "Measurement.h"
#include "Sensor.h"
#include <sstream>
#include <algorithm>
#include <iostream>

using Eigen::VectorXd;
using std::string;

// static initializer of the factory instance
MeasurementFactory *MeasurementFactory::instance_ = 0;

Measurement *MeasurementFactory::CreateMeasurement(std::istringstream &iss, const SensorContainer &sensors) {

    string sensor_type;
    const Sensor *sensor = 0;
    SensorContainer::iterator si;
    long timestamp;

    iss >> sensor_type;

    try {
        sensor = sensors.at(sensor_type);
    } catch (const std::out_of_range &e) {
        // discard measurement because sensor not valid
   	//std::cout << "Invalid sensor type" << std::endl;
        return 0;
    }

    VectorXd measurements(sensor->df_);

    if (dynamic_cast<const LidarSensor*>(sensor) != 0) {
      float px;
      float py;
      
      iss >> px;
      iss >> py;

      measurements << px, py;

    } else if (dynamic_cast<const RadarSensor*>(sensor) != 0) {
      float rho;
      float theta;
      float rho_dot;

      iss >> rho;
      iss >> theta;
      iss >> rho_dot;

      measurements << rho, theta, rho_dot;
    }   

    iss >> timestamp;

    Measurement *m = CreateMeasurement(sensor, timestamp, measurements);

    // read ground truth data and store it with the measurement
    if (m) {

        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
	m->ground_truth_ = VectorXd(4);
        m->ground_truth_  << x_gt, y_gt, vx_gt, vy_gt;
    }

    return m;
}

Measurement *MeasurementFactory::CreateMeasurement(const Sensor *sensor, long timestamp, VectorXd &measurements) {

    Measurement *m = 0; 

    if (dynamic_cast<const LidarSensor*>(sensor) != 0) {
        m = new LidarMeasurement(sensor, timestamp, measurements);
    } else if (dynamic_cast<const RadarSensor*>(sensor) != 0) {
        m = new RadarMeasurement(sensor, timestamp, measurements);
    }

    return m;
}

MeasurementFactory *MeasurementFactory::GetInstance() {
    if (instance_ == 0)
	    instance_ = new MeasurementFactory();
    return instance_;
}

