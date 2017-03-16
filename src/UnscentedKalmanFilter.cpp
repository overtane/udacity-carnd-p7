#include <iostream>
#include <math.h>
#include "UnscentedKalmanFilter.h"
#include "Tools.h"


static double norm_angle(double a) {
    static const double PI2 = 2.*M_PI;
    a = fmod(a,PI2); // force between [0, 2*PI)
    return (a > M_PI) ? a - PI2 : a; // [-PI, PI)
    //return a - ceil((a-M_PI)/(PI2))*PI2;
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
  P_ << 1, 0,    0,      0, 0,
        0, 1,    0,      0, 0,
	0, 0, 1000,      0, 0,
	0, 0,    0, M_PI/4, 0,
	0, 0,    0,      0, M_PI/8;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; 

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
 * @param {MeasurementPackage} mpage The latest measurement data of
 * either radar or laser.
 */
double UnscentedKalmanFilter::ProcessMeasurement(MeasurementPackage mp) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {

    previous_timestamp_ = mp.timestamp_;

    // initialize the state vector
    if (mp.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = mp.raw_measurements_[0];
      double phi = mp.raw_measurements_[1];
      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
      is_initialized_ = true;

    } else if (mp.sensor_type_ == MeasurementPackage::LASER) {
      x_ << mp.raw_measurements_[0], mp.raw_measurements_[1], 0, 0, 0;
      is_initialized_ = true;

    } else {
      std::cout << "Invalid sensor type" << std::endl;
    }
    return 0.0;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements, in seconds
  double dt = (mp.timestamp_ - previous_timestamp_) / 1000000.0;	
  
  // skip prediction, if measurement has (about) the same timestamp as previous
  // timestamps are microseconds
  //if (mp.timestamp_ > previous_timestamp_) {
  //  dt = 1.0;
  //} else 
  //  dt = 0.0;
      
  Prediction(dt);
  previous_timestamp_ = mp.timestamp_;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  double nis = 0.0; 

  if (mp.sensor_type_ == MeasurementPackage::RADAR) {
    
    nis = UpdateRadar(mp);

  } else if (mp.sensor_type_ == MeasurementPackage::LASER) {

    nis = UpdateLidar(mp);
  }

  return nis;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UnscentedKalmanFilter::Prediction(double delta_t) {

    MatrixXd Xsig_aug(n_aug_, n_sigma_);
 
    GenerateAugmentedSigmaPoints(x_, P_, Xsig_aug);
  
    SigmaPointPrediction(delta_t, Xsig_aug, Xsig_pred_);
    //std::cout << "Xsig_pred:" << Xsig_pred_ << std::endl;
  
    PredictMeanAndCovariance(Xsig_pred_, x_, P_);
    //std::cout << "x:" << x_.transpose() << std::endl;
    //std::cout << "P:" << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} mpage
 */
double UnscentedKalmanFilter::UpdateLidar(MeasurementPackage mp) {

  VectorXd y = mp.raw_measurements_;

  int n_y = y.rows();

  VectorXd y_pred(n_y);
  MatrixXd S(n_y,n_y);
  MatrixXd Ysig(n_y, n_sigma_);

  PredictLidarMeasurement(Xsig_pred_, Ysig, y_pred, S);
  UpdateState(y, y_pred, S, Xsig_pred_, Ysig, x_, P_);
  
  //std::cout << "x:" << x_.transpose() << std::endl;
  //std::cout << "P:" << P_ << std::endl;

  return Tools::NIS(y, y_pred, S);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} mpage
 */
double UnscentedKalmanFilter::UpdateRadar(MeasurementPackage mp) {

  VectorXd z = mp.raw_measurements_;

  int n_z = z.rows();

  VectorXd z_pred(n_z);
  MatrixXd S(n_z,n_z);
  MatrixXd Zsig(n_z, n_sigma_);

  PredictRadarMeasurement(Xsig_pred_, Zsig, z_pred, S);
  //std::cout << "z_pred:" << z_pred.transpose() << std::endl;
  //std::cout << "S:" << S << std::endl;
  UpdateState(mp.raw_measurements_, z_pred, S, Xsig_pred_, Zsig, x_, P_);
  //std::cout << "x:" << x_.transpose() << std::endl;
  //std::cout << "P:" << P_ << std::endl;
  
  return Tools::NIS(z, z_pred, S);
}

void UnscentedKalmanFilter::GenerateSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig) {

    int n_x = x.rows();

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();

    //set first column of sigma point matrix
    Xsig.col(0)  = x;

    double s = sqrt(lambda_+n_x_);

    // set remaining sigma points
    for (int i = 0; i < n_x; i++) {
	Xsig.col(i+1)     = x + s * A.col(i);
        Xsig.col(i+1+n_x) = x - s * A.col(i);
    }
}

bool UnscentedKalmanFilter::GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug) {

    bool ret = true;
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
    P_aug(n_aug_-1,n_aug_-1) = std_yawdd_*std_yawdd_;

    // create square root matrix
    //MatrixXd L = P_aug.llt().matrixL();
    Eigen::LLT<MatrixXd> lltOfPaug(P_aug); // compute the Cholesky decomposition of P_aug
    if(lltOfPaug.info() == Eigen::NumericalIssue) {
        std::cout << "LLT failed!" << std::endl;
    }	
    MatrixXd L = lltOfPaug.matrixL(); 
    //    std::cout << "P_aug:" << P_aug << std::endl;
    //	ret = false;
    //
    //} else {
    //    MatrixXd L = lltOfPaug.matrixL(); 

        //std::cout << "P_aug:" << P_aug << std::endl;
        //std::cout << "L:" << L << std::endl;
        //std::cout << "L*Lt:" << L * L.transpose() << std::endl;
        //
        // create augmented sigma points
        
	Xsig_aug.col(0) = x_aug;
        
	double s = sqrt(lambda_+n_aug_);

        for (int i=0; i<n_aug_; i++) {
	    int j = i+1;
	    int k = j+n_aug_;
            Xsig_aug.col(j) = x_aug + s * L.col(i);
            Xsig_aug.col(k) = x_aug - s * L.col(i);
	    // normalize
            //Xsig_aug(3,j)   = norm_angle(Xsig_aug(3,j));
            //Xsig_aug(3,k)   = norm_angle(Xsig_aug(3,k));
            //Xsig_aug(4,j)   = norm_angle(Xsig_aug(4,j));
            //Xsig_aug(4,k)   = norm_angle(Xsig_aug(4,k));
        }
    //}

    return ret;
}

void UnscentedKalmanFilter::SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred) {

  for (int i=0; i<n_sigma_; i++) {
#if 1
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
      px_p = px + v/yawd * (sin(yaw+yawd*delta_t) - sin(yaw));
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
    //yaw_p = norm_angle(yaw_p);
    yawd_p = yawd_p + nuyawdd*delta_t;
    //yawd_p = norm_angle(yawd_p);

    // write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
#else
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
    //
    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
#endif
  }
}

void UnscentedKalmanFilter::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) {

  x.fill(0.0);
  P.fill(0.0);

  // predicted state mean
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points
      x = x + weights_(i) * Xsig_pred.col(i);
  }
  //x(3) = norm_angle(x(3));
  //x(4) = norm_angle(x(4));

  // predicted state covariance matrix
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = norm_angle(x_diff(3));
    //x_diff(4) = norm_angle(x_diff(4));

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //for (int i=0; i<P.cols(); i++) {
  //     P(3,i) = norm_angle(P(3,i));
  //     P(4,i) = norm_angle(P(4,i));
  //}
}
 
void UnscentedKalmanFilter::PredictLidarMeasurement(const MatrixXd &Xsig_pred, MatrixXd &Ysig, VectorXd &y_pred, MatrixXd &S) {

    int n_y = y_pred.rows();

    // Initialize output
    y_pred.fill(0.0);
    S.fill(0.0);

    // transform sigma points into measurement space
    for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points
	// prepare column i
	Ysig(0,i) = Xsig_pred(0,i);
	Ysig(1,i) = Xsig_pred(1,i);

        // add to mean predicted measurement
        y_pred = y_pred + weights_(i) * Ysig.col(i);
    }

    // measurement covariance matrix S
    for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points
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

  z_pred.fill(0.0);
  S.fill(0.0);

  // transform sigma points into measurement space
  for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points

        // extract values for better readibility
        double px  = Xsig_pred(0,i);
        double py  = Xsig_pred(1,i);
        double v   = Xsig_pred(2,i);
        double yaw = Xsig_pred(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
	// prepare column i
        Zsig(0,i) = sqrt(px*px + py*py);          // r
        if (fabs(Zsig(0,i))<0.0001)
	    Zsig(0,i) = 0.0001;

	Zsig(1,i) = atan2(py,px);                 // phi
        Zsig(2,i) = (px*v1 + py*v2 ) / Zsig(0,i); // r_dot

        // add column to mean predicted measurement
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }
    //z_pred(1) = norm_angle(z_pred(1));

    // measurement covariance matrix S
    for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = norm_angle(z_diff(1));

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R(n_z,n_z);
    R <<    std_radr_*std_radr_, 0,                       0,
            0,                   std_radphi_*std_radphi_, 0,
            0,                   0,                       std_radrd_*std_radrd_;

    S = S + R;

    //for (int i=0; i<S.cols(); i++) {
    //    S(1,i) = norm_angle(S(1,i));
    //}
}
 
void UnscentedKalmanFilter::UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, 
		const MatrixXd &Xsig_pred, const MatrixXd &Zsig, VectorXd &x, MatrixXd &P) {


   int n_z = z_pred.rows();	
   int n_x = x.rows();

   // calculate cross correlation matrix
   MatrixXd Tc(n_x, n_z);

   Tc.fill(0.0);

   for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points

       // residual
       VectorXd z_diff = Zsig.col(i) - z_pred;
       if (n_z == 3) // angle in z;
           z_diff(1) = norm_angle(z_diff(1));

       // state difference
       VectorXd x_diff = Xsig_pred.col(i) - x;
       x_diff(3) = norm_angle(x_diff(3));
       //x_diff(4) = norm_angle(x_diff(4));

       Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   //for (int i=0; i<Tc.cols(); i++) {
   //	   Tc(3,i) = norm_angle(Tc(3,i));
   //	   Tc(4,i) = norm_angle(Tc(4,i));
   //}

   // Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   // residual
   VectorXd z_diff = z - z_pred;
   if (n_z == 3)
       z_diff(1) = norm_angle(z_diff(1));

   // update state mean and covariance matrix
   x = x + K * z_diff;
   //x(3) = norm_angle(x(3));
   //x(4) = norm_angle(x(4));
   P = P - K*S*K.transpose();
   //for (int i=0; i<n_x_; i++) {
   //    P(3,i) = norm_angle(P(3,i));
   //    P(4,i) = norm_angle(P(4,i));
   //}

#if 0
   std::cout << "z:" << z.transpose() << std::endl;
   std::cout << "z_pred:" << z_pred.transpose() << std::endl;
   std::cout << "Xsig_pred:" << Xsig_pred << std::endl;
   std::cout << "Zsig:" << Zsig << std::endl;
   std::cout << "Tc:" << Tc << std::endl;
   std::cout << "S:" << S << std::endl;
   std::cout << "S inverse:" << S.inverse() << std::endl;
   std::cout << "K:" << K << std::endl;
   std::cout << "K transpose:" << K.transpose() << std::endl;
   std::cout << "P:" << P << std::endl;
   std::cout << "x:" << x.transpose() << std::endl;
#endif
}



#if 0
  Xsig_pred_ <<
            5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
            1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
            0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  x_ <<
            5.93637,
            1.49035,
            2.20528,
            0.536853,
            0.353577;
  P_ <<
            0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
           -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
            0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
           -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
           -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
  Zsig <<
            6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
            0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
            2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
  z_pred <<
            6.12155,
            0.245993,
            2.10313;
  S <<
            0.0946171, -0.000139448,   0.00407016,
           -0.000139448,  0.000617548, -0.000770652,
            0.00407016, -0.000770652,    0.0180917;

  VectorXd z(3);
  z <<
            5.9214,
            0.2187,
            2.0062;
  //
#endif

#if 0
      delta_t = 0.1;
      Xsig_aug <<
          5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
          1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
          2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
          0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
          0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
          0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
          0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;
#endif

#if 0
    x_ <<  5.7441,
           1.3800,
           2.2049,
           0.5015,
           0.3528;

    P_ <<  0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
    MatrixXd Xsig_aug(n_aug_, n_sigma_);
    MatrixXd Xsig_aug_expected(n_aug_, n_sigma_);

    GenerateAugmentedSigmaPoints(x_, P_, Xsig_aug);

    Xsig_aug_expected <<
        5.7441, 5.85768, 5.7441,  5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441
        1.38,   1.34566, 1.52806, 1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
	    2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
	      0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
	        0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
		       0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
		              0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641

#endif


