#include <iostream>
#include "UnscentedKalmanFilter.h"


static double norm_angle(double a) {
    return  a - ceil((a-M_PI)/(2.*M_PI))*2.*M_PI;
    //static const double M_PI_TWICE = 2. * M_PI;
    //while (a> M_PI) a-=M_PI_TWICE;
    //while (a<-M_PI) a+=M_PI_TWICE;
    //return a;
}	

/**
 * Initializes Unscented Kalman filter
 */
UnscentedKalmanFilter::UnscentedKalmanFilter() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
	0, 0, 1, 0, 0,
	0, 0, 0, 1, 0,
	0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = .5; 

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  previous_timestamp_ = 0;

  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_; // TODO: how is lambda defined?
  n_sigma_ = 2 * n_aug_ + 1;

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_); 

  // set weights
  weights_ = VectorXd(n_sigma_);
  weights_(0)= lambda_/(lambda_+n_aug_);
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights
    weights_(i) = .5/(n_aug_+lambda_);
  }
  
}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UnscentedKalmanFilter::ProcessMeasurement(MeasurementPackage meas_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    previous_timestamp_ = meas_pack.timestamp_;

    // initialize the state vector
    if (meas_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float ro = meas_pack.raw_measurements_[0];
      float phi = meas_pack.raw_measurements_[1];
      x_ << ro * cos(phi), ro * sin(phi), 0, 0, 0;
      //x_ << 0, 0, 0, 0, 0;
      is_initialized_ = true;

    } else if (meas_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_pack.raw_measurements_[0], meas_pack.raw_measurements_[1], 0, 0, 0;
      is_initialized_ = true;

    } else {
      std::cout << "Invalid sensor type" << std::endl;
    }
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements, in seconds
  float dt = (meas_pack.timestamp_ - previous_timestamp_) / 1000000.0;	
    
  // skip prediction, if measurement has (about) the same timestamp as previous
  // timestamps are microseconds
  if (previous_timestamp_ < meas_pack.timestamp_ - 100) {
    Prediction(dt);
    // update prediction timestamp only if we did prediction
    previous_timestamp_ = meas_pack.timestamp_;
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (meas_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    UpdateRadar(meas_pack);

  } else if (meas_pack.sensor_type_ == MeasurementPackage::LASER) {

    UpdateLidar(meas_pack);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UnscentedKalmanFilter::Prediction(double delta_t) {

  MatrixXd Xsig_aug(n_aug_, n_sigma_);

  GenerateAugmentedSigmaPoints(x_, P_, Xsig_aug);
  std::cout << Xsig_aug << std::endl;
  SigmaPointPrediction(delta_t, Xsig_aug, Xsig_pred_);
  //std::cout << Xsig_pred_ << std::endl;
  PredictMeanAndCovariance(Xsig_pred_, x_, P_);
  //std::cout << "x:" << x_.transpose() << std::endl;
  //std::cout << "P:" << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UnscentedKalmanFilter::UpdateLidar(MeasurementPackage meas_pack) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  int n_y = meas_pack.raw_measurements_.rows();

  VectorXd y_pred(n_y);
  MatrixXd S(n_y,n_y);
  MatrixXd Ysig(n_y, n_sigma_);

  PredictLidarMeasurement(Xsig_pred_, Ysig, y_pred, S);
  UpdateState(meas_pack.raw_measurements_, y_pred, S, Xsig_pred_, Ysig, x_, P_);
  std::cout << "x:" << x_.transpose() << std::endl;
  std::cout << "P:" << P_ << std::endl;
  // TODO You'll also need to calculate the lidar NIS.
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UnscentedKalmanFilter::UpdateRadar(MeasurementPackage meas_pack) {

  int n_z = meas_pack.raw_measurements_.rows();

  VectorXd z_pred(n_z);
  MatrixXd S(n_z,n_z);
  MatrixXd Zsig(n_z, n_sigma_);

  PredictRadarMeasurement(Xsig_pred_, Zsig, z_pred, S);
  UpdateState(meas_pack.raw_measurements_, z_pred, S, Xsig_pred_, Zsig, x_, P_);
  std::cout << "x:" << x_.transpose() << std::endl;
  std::cout << "P:" << P_ << std::endl;
  
  // TODO You'll also need to calculate the radar NIS.
}

void UnscentedKalmanFilter::GenerateSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig) {

    int n_x = x.rows();

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();

    //set first column of sigma point matrix
    Xsig.col(0)  = x;

    // set remaining sigma points
    for (int i = 0; i < n_x; i++) {
	Xsig.col(i+1)     = x + sqrt(lambda_+n_x) * A.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda_+n_x) * A.col(i);
    }
}

void UnscentedKalmanFilter::GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug) {

    int n_x = x.rows();
    
    // create augmented mean vector
    VectorXd x_aug(n_aug_);
  
    // create augmented state covariance
    MatrixXd P_aug(n_aug_, n_aug_);

    // create augmented mean state
    x_aug << x, 0, 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x,n_x) = P;
    P_aug(n_aug_-2,n_aug_-2) = std_a_*std_a_;
    P_aug(n_aug_-1,n_aug_-1) = std_yawdd_ * std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    std::cout << "P_aug:" << P_aug << std::endl;
    std::cout << "L:" << L << std::endl;
    std::cout << "L*Lt:" << L * L.transpose() << std::endl;
    // create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    int a = lambda_+n_aug_;
    for (int i=0; i<n_aug_; i++) {
        Xsig_aug.col(i+1)        = x_aug + sqrt(a) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(a) * L.col(i);
    }
}

void UnscentedKalmanFilter::SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred) {

  for (int i=0; i<n_sigma_; i++) {

    // extract values for better readability
    double px      = Xsig_aug(0,i);
    double py      = Xsig_aug(1,i);
    double v       = Xsig_aug(2,i);
    double yaw     = Xsig_aug(3,i);
    double yawd    = Xsig_aug(4,i);
    double nua     = Xsig_aug(5,i);
    double nuyawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    if (fabs(yawd) > 0.001) {
      px_p = px + v/yawd * (sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = py + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
      px_p = px + v*delta_t*cos(yaw);
      py_p = py + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    double dts = delta_t * delta_t;
    px_p = px_p + 0.5*nua*dts*cos(yaw);
    py_p = py_p + 0.5*nua*dts*sin(yaw);
    v_p = v_p + nua*delta_t;

    yaw_p = yaw_p + 0.5*nuyawdd*dts;
    yawd_p = yawd_p + nuyawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }
}

void UnscentedKalmanFilter::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) {

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  // iterate over sigma points
      x = x + weights_(i) * Xsig_pred.col(i);
  }
  x(3) = norm_angle(x(3));

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = norm_angle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose() ;
  }
}
 
void UnscentedKalmanFilter::PredictLidarMeasurement(const MatrixXd &Xsig_pred, MatrixXd &Ysig, VectorXd &y_pred, MatrixXd &S) {

    int n_y = y_pred.rows();

    // transform sigma points into measurement space
    for (int i = 0; i < n_sigma_; i++) {  // 2n+1 sigma points

	Ysig(0, i) = Xsig_pred(0,i);
	Ysig(1, i) = Xsig_pred(1,i);
    }

    // mean predicted measurement
    y_pred.fill(0.0);
    for (int i=0; i < n_sigma_; i++) {
        y_pred = y_pred + weights_(i) * Ysig.col(i);
    }

    // measurement covariance matrix S
    S.fill(0.0);
    for (int i=0; i < n_sigma_; i++) {  // 2n+1 sigma points
        // residual
        VectorXd y_diff = Ysig.col(i) - y_pred;
        S = S + weights_(i) * y_diff * y_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_y,n_y);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    S = S + R;
}
void UnscentedKalmanFilter::PredictRadarMeasurement(const MatrixXd &Xsig_pred, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

  int n_z = z_pred.rows();

  // transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; i++) {  // 2n+1 sigma points

        // extract values for better readibility
        double px  = Xsig_pred(0,i);
        double py  = Xsig_pred(1,i);
        double v   = Xsig_pred(2,i);
        double yaw = Xsig_pred(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(px*px + py*py);              // r
        if (fabs(Zsig(0,i))>0.001) {
	    Zsig(1,i) = atan2(py,px);                 // phi
            Zsig(2,i) = (px*v1 + py*v2 ) / Zsig(0,i); // r_dot
	} else {
	    Zsig(1,i) = 0.0;  // phi
            Zsig(2,i) = 0.0;  // r_dot
	}
    }

    // mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i < n_sigma_; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // measurement covariance matrix S
    S.fill(0.0);
    for (int i=0; i < n_sigma_; i++) {  // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = norm_angle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;

    S = S + R;
}
 
void UnscentedKalmanFilter::UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, 
		const MatrixXd &Xsig_pred, const MatrixXd &Zsig, VectorXd &x, MatrixXd &P) {

   int n_z = z_pred.rows();	
   int n_x = x.rows();

   // calculate cross correlation matrix
   MatrixXd Tc = MatrixXd(n_x, n_z);
   Tc.fill(0.0);
   for (int i=0; i < n_sigma_; i++) {  // 2n+1 sigma points

       // residual
       VectorXd z_diff = Zsig.col(i) - z_pred;
       z_diff(1) = norm_angle(z_diff(1));

       // state difference
       VectorXd x_diff = Xsig_pred.col(i) - x;
       x_diff(3) = norm_angle(x_diff(3));

       Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   //residual
   VectorXd z_diff = z - z_pred;
   z_diff(1) = norm_angle(z_diff(1));

   // update state mean and covariance matrix
   x = x + K * z_diff;
   x(3) = norm_angle(x(3));
   P = P - K*S*K.transpose();
}
