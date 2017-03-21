#include "Eigen/Dense"
#include "Measurement.h"
#include "Tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


std::ostream& operator<<(std::ostream& strm, const Measurement &m) { return m.toString(strm); }

Measurement::Measurement(const Sensor *sensor, long timestamp, const VectorXd& measurements) :
        sensor_(sensor),
	timestamp_(timestamp),
	measurements_(measurements),
	nis_(0.0)
{
    //std::cout << measurements_.transpose() << std::endl;
}

std::ostream& Measurement::toString(std::ostream &strm) const {
    // output the estimation
    for (int i=0; i<estimate_.rows(); i++)
        strm << "\t" << estimate_(i);

    strm << "\t" << nis_; // nis

    // output the ground truth packages
    for (int i=0; i<ground_truth_.rows(); i++)
        strm << "\t" << ground_truth_(i);

    strm << "\n";

    return strm;
}

double Measurement::NIS(const VectorXd &z_pred, const MatrixXd &S) {

  // TODO check dimensions
  VectorXd z_diff = measurements_ - z_pred;
  nis_ = z_diff.transpose() * S.inverse() * z_diff;
  return nis_;
}

LidarMeasurement::LidarMeasurement(const Sensor *sensor, long timestamp, const VectorXd& measurements) :
        Measurement(sensor, timestamp, measurements)
{}


VectorXd LidarMeasurement::InitializeState(int n) const {

    VectorXd x(n);
    // initialize the state vector
    x.fill(1.0);
    x(0) = (fabs(measurements_[0]) < 0.01) ? 0.01 : measurements_[0];
    x(1) = (fabs(measurements_[1]) < 0.01) ? 0.01 : measurements_[1];
    return x;
}

std::ostream& LidarMeasurement::toString(std::ostream &strm) const {
    // actual measurement
    strm << measurements_(0) << "\t"
	 << measurements_(1);
 
    return Measurement::toString(strm);
}


RadarMeasurement::RadarMeasurement(const Sensor *sensor, long timestamp, const VectorXd& measurements) :
        Measurement(sensor, timestamp, measurements)
{}

VectorXd RadarMeasurement::InitializeState(int n) const {

    VectorXd x(n);
    // initialize the state vector
    x.fill(1.0);
    double rho = measurements_[0];
    double phi = measurements_[1];
    x(0) = (fabs(rho)<0.001) ? 0.01 : rho * cos(phi);
    x(1) = (fabs(rho)<0.001) ? 0.01 : rho * sin(phi);
    x(2) = measurements_[2];
    x(3) = phi;
    return x;
}

std::ostream& RadarMeasurement::toString(std::ostream &strm) const {
    float rho = measurements_(0);
    float phi = measurements_(1);

    strm << rho*cos(phi) << "\t"
         << rho*sin(phi);

    return Measurement::toString(strm);
}


