#include "Eigen/Dense"
#include "Measurement.h"
#include "Tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


MatrixXd LidarMeasurement::R_;
MatrixXd RadarMeasurement::R_;


std::ostream& operator<<(std::ostream& strm, const Measurement &m) { return m.toString(strm); }

Measurement::Measurement(long timestamp, const VectorXd& measurements) :
	timestamp_(timestamp),
	measurements_(measurements),
	nis_(0.0)
{
    //std::cout << measurements_.transpose() << std::endl;
}

void Measurement::SetR(const VectorXd& r, MatrixXd& R) {
	R = MatrixXd(r.rows(), r.rows());
	R.fill(0.0);
	for (int i=0; i<R.rows(); i++) {
	    R(i,i) = r(i)*r(i);
	}
}

LidarMeasurement::LidarMeasurement(long timestamp, const VectorXd& measurements) :
        Measurement(timestamp, measurements)
{}


void LidarMeasurement::SetR(const VectorXd& r) {
	Measurement::SetR(r, LidarMeasurement::R_);
}

VectorXd LidarMeasurement::InitializeState(int n) const {

    VectorXd x(n);
    // initialize the state vector
    x.fill(0.0);
    x(0) = measurements_[0];
    x(1) = measurements_[1];
    return x;
}

void LidarMeasurement::PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const {

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
    for (int i=0; i<n_sigma; i++) {  // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    S = S + GetR();
}

const MatrixXd& LidarMeasurement::GetR() const {
    return LidarMeasurement::R_;
}

std::ostream& LidarMeasurement::toString(std::ostream &strm) const {
    return strm << measurements_(0) << "\t" << measurements_(1) << "\t";
}



RadarMeasurement::RadarMeasurement(long timestamp, const VectorXd& measurements) :
        Measurement(timestamp, measurements)
{}

void RadarMeasurement::SetR(const VectorXd& r) {
	Measurement::SetR(r, RadarMeasurement::R_);
}

VectorXd RadarMeasurement::InitializeState(int n) const {

    VectorXd x(n);
    // initialize the state vector
    x.fill(0.0);
    double rho = measurements_[0];
    double phi = measurements_[1];
    x(0) = rho * cos(phi);
    x(1) = rho * sin(phi);
    return x;
}

void RadarMeasurement::PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const {

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
        Zsig(2,i) = (px*v1 + py*v2 ) / Zsig(0,i); // r_dot

        // add column to mean predicted measurement
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    for (int i=0; i<n_sigma; i++) {  // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = Tools::NormalizeAngle(z_diff(1));

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    S = S + GetR();
}

const MatrixXd& RadarMeasurement::GetR() const {
    return RadarMeasurement::R_;
}

std::ostream& RadarMeasurement::toString(std::ostream &strm) const {
      float rho = measurements_(0);
      float phi = measurements_(1);
      return strm << rho*cos(phi) << "\t" << rho*sin(phi) << "\t";
}



