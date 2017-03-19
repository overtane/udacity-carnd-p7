#include <iostream>
#include <math.h>
#include "UnscentedKalmanFilter.h"
#include "Tools.h"

static const double PI2 = 2.*M_PI;

#define NORM_ANGLE_LOOP
#ifdef NORM_ANGLE_LOOP
static double norm_angle(double a) {
   if (fabs(a) > M_PI) {
     cout << "NORM_ANGLE: " << a << endl;
     while (a > M_PI) a -= PI2;
     while (a < -M_PI) a += PI2;
   }
   return a;
}
#endif

//#define NORM_ANGLE_CEIL
#ifdef NORM_ANGLE_CEIL
static double norm_angle(double a) {
    return a - ceil((a-M_PI)/(PI2))*PI2;
}
#endif

/**
 * Initializes Unscented Kalman filter
 */
UnscentedKalmanFilter::UnscentedKalmanFilter() {
}

UnscentedKalmanFilter::UnscentedKalmanFilter(double std_a, double std_yawdd) :
    is_initialized_(false),
    n_x_(5),
    std_a_(std_a),
    std_yawdd_(std_yawdd)  
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

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

  n_aug_ = n_x_ + 2;
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

void UnscentedKalmanFilter::InitializeMeasurement(MeasurementPackage mp) {
    
    previous_measurement_ = mp;

    // initialize the covariance matrix
    P_ = MatrixXd::Identity(n_x_, n_x_); 
    x_.fill(0.0);

    // initialize the state vector
    if (mp.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = mp.raw_measurements_[0];
      double phi = mp.raw_measurements_[1];
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      is_initialized_ = true;

    } else if (mp.sensor_type_ == MeasurementPackage::LASER) {
      x_(0) = mp.raw_measurements_[0];
      x_(1) = mp.raw_measurements_[1];
      is_initialized_ = true;

    } else {
      // Invalid data
      throw std::invalid_argument("Invalid sensor type in measurement");
    }
    return;
}
/**
 * @param {MeasurementPackage} mpage The latest measurement data of
 * either radar or laser.
 */
double UnscentedKalmanFilter::ProcessMeasurement(MeasurementPackage mp) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
      InitializeMeasurement(mp);
      return 0.0;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements, in seconds
  double dt = (mp.timestamp_ - previous_measurement_.timestamp_) / 1000000.0;	

  try {
      Prediction(dt);
  } catch (std::range_error e) {
      // If convariance matrix is non positive definite (because of numerical instability),
      // restart the filter using previous measurement as initializer.
      InitializeMeasurement(previous_measurement_);
      // Redo prediction using the current measurement
      // We don't get exception this time, because intial P (identity) is positive definite.
      Prediction(dt);
  }

  previous_measurement_ = mp;

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
    PredictMeanAndCovariance(Xsig_pred_, x_, P_);
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
  UpdateState(mp.raw_measurements_, z_pred, S, Xsig_pred_, Zsig, x_, P_);
  
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
 
    // Take matrix square root
    // 1. compute the Cholesky decomposition of P_aug
    Eigen::LLT<MatrixXd> lltOfPaug(P_aug); 
    if (lltOfPaug.info() == Eigen::NumericalIssue) {
	// if decomposition fails, we have numerical issues
        std::cout << "LLT failed!" << std::endl;
	Eigen::EigenSolver<MatrixXd> es(P_aug);
	cout << "P_aug:" << P_aug << endl;
	cout << "Eigenvalues of P_aug:" << endl << es.eigenvalues() << endl;
	throw std::range_error("LLT failed");
    }
	//Eigen::EigenSolver<MatrixXd> es(P_aug);
	//cout << "P_aug:" << P_aug << endl;
	//cout << "Eigenvalues of P_aug:" << endl << es.eigenvalues() << endl;

    // 2. get the lower triangle
    MatrixXd L = lltOfPaug.matrixL(); 

    Xsig_aug.col(0) = x_aug;
        
    double s = sqrt(lambda_+n_aug_);

    for (int i=0; i<n_aug_; i++) {
	int j = i+1;
	int k = j+n_aug_;
        Xsig_aug.col(j) = x_aug + s * L.col(i);
        Xsig_aug.col(k) = x_aug - s * L.col(i);
    }

    return ret;
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

  x.fill(0.0);
  P.fill(0.0);

  // predicted state mean
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points
      x = x + weights_(i) * Xsig_pred.col(i);
  }
  x(3) = norm_angle(x(3));

  // predicted state covariance matrix
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = norm_angle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //Eigen::EigenSolver<MatrixXd> es(P);
  //cout << "Eigenvalues of P pred:" << endl << es.eigenvalues() << endl;
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
	// prepare column
        Zsig(0,i) = sqrt(px*px + py*py);          // r
        if (fabs(Zsig(0,i))<0.0001) // prevent div by zero
	    Zsig(0,i) = 0.0001;

	Zsig(1,i) = atan2(py,px);                 // phi
        Zsig(2,i) = (px*v1 + py*v2 ) / Zsig(0,i); // r_dot

        // add column to mean predicted measurement
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

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

       Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   // residual
   VectorXd z_diff = z - z_pred;
   if (n_z == 3)
       z_diff(1) = norm_angle(z_diff(1));

   // update state mean and covariance matrix
   x = x + K * z_diff;
   x(3) = norm_angle(x(3));
   P = P - K*S*K.transpose();

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
   std::cout << "P inverse:" << P.inverse() << std::endl;
   std::cout << "x:" << x.transpose() << std::endl;
   Eigen::EigenSolver<MatrixXd> es(P);
   cout << "Eigenvalues of P:" << endl << es.eigenvalues() << endl;
#endif
}
