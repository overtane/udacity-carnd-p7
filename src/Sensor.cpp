#include "Eigen/Dense"
#include "Sensor.h"
#include "Tools.h"
#include <string>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

Sensor::Sensor(string name, const VectorXd& noise, int df) :
	name_(name),
	df_(df)
{
    SetR(noise);
}

void Sensor::SetR(const VectorXd& r) {
    R_ = MatrixXd(r.rows(), r.rows());
    R_.fill(0.0);
    for (int i=0; i<R_.rows(); i++) {
        R_(i,i) = r(i)*r(i);
    }
}

const MatrixXd &Sensor::GetR() const {
    return R_;
}

LidarSensor::LidarSensor(string name, const VectorXd& noise) :
        Sensor(name, noise, 2)
{}

void LidarSensor::NormalizeMeasurement(VectorXd &x) const {
    (void) x;
}

void LidarSensor::PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, bool modified) const {

    int n_z = z_pred.rows();
    int n_sigma = Zsig.cols();

    // Initialize output
    z_pred.fill(0.0);
    S.fill(0.0);

    // transform sigma points into measurement space
    for (int i=0; i<n_sigma; i++) {  // 2n+1 sigma points
	// prepare column i
	Zsig(0,i) = Xsig_pred(0,i);
	Zsig(1,i) = Xsig_pred(1,i);

        // add to mean predicted measurement
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    for (int i=(modified)?1:0; i<n_sigma; i++) {  // 2n+1 sigma points
        // residual
	VectorXd z_diff;
	if (modified)
            z_diff = Zsig.col(i) - Zsig.col(0);
	else
	    z_diff = Zsig.col(i) - z_pred;

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    S = S + GetR();
}


RadarSensor::RadarSensor(string name, const VectorXd& noise) :
        Sensor(name, noise, 3)
{}

void RadarSensor::NormalizeMeasurement(VectorXd &x) const {
    x(1) = Tools::NormalizeAngle(x(1));
}

void RadarSensor::PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, bool modified) const {

  int n_z = z_pred.rows();
  int n_sigma = Zsig.cols();

  z_pred.fill(0.0);
  S.fill(0.0);

  // transform sigma points into measurement space
  for (int i=0; i<n_sigma; i++) {  // 2n+1 sigma points

        // extract values for better readibility
        double px  = Xsig_pred(0,i);
        double py  = Xsig_pred(1,i);
        double v   = Xsig_pred(2,i);
        double yaw = Xsig_pred(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
	// prepare column
        Zsig(0,i) = sqrt(px*px + py*py);          // r
        if (fabs(Zsig(0,i))<0.0001) // prevent div by zero
	    Zsig(0,i) = 0.0001;

	Zsig(1,i) = atan2(py,px);                 // phi
        Zsig(2,i) = (px*v1 + py*v2) / Zsig(0,i); // r_dot

        // add column to mean predicted measurement
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    for (int i=(modified)?1:0; i<n_sigma; i++) {  // 2n+1 sigma points
        // residual
        VectorXd z_diff;
	if (modified)
            z_diff = Zsig.col(i) - Zsig.col(0);
	else
	    z_diff = Zsig.col(i) - z_pred;

        z_diff(1) = Tools::NormalizeAngle(z_diff(1));

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    S = S + GetR();
}



