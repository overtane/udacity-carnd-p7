#include <iostream>
#include <math.h>
#include "UnscentedKalmanFilter.h"
#include "Tools.h"

using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UnscentedKalmanFilter::UnscentedKalmanFilter(double std_a, double std_yawdd, int debug) :
    std_a_(std_a),
    std_yawdd_(std_yawdd),
    debug_(debug),	
    is_initialized_(false),
    n_x_(5)
{
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

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

void UnscentedKalmanFilter::InitializeMeasurement(const Measurement *m) {
    
    // initialize the covariance matrix
    P_ = MatrixXd::Identity(n_x_, n_x_); 
    // initialize the state vector
    x_ = m->InitializeState(n_x_);
    return;
}
/**
 * @param {MeasurementPackage} mp The latest measurement data of
 * either radar or laser.
 */
double UnscentedKalmanFilter::ProcessMeasurement(Measurement *m) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
      InitializeMeasurement(m);
      previous_measurement_ = m;
      is_initialized_ = true;
      return 0.0;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements, in seconds
  double dt = (m->timestamp_ - previous_measurement_->timestamp_) / 1000000.0;	

  try {
      Prediction(dt);
  } catch (std::range_error e) {
      // If convariance matrix is non positive definite (because of numerical instability?),
      // restart the filter using previous measurement as an initializer.
      InitializeMeasurement(previous_measurement_);
      // Redo prediction using the current measurement
      // We don't get exception this time, because intial P (identity) is positive definite.
      Prediction(dt);
  }

  previous_measurement_ = m;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  m->nis_ = Update(m);

  return m->nis_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UnscentedKalmanFilter::Prediction(double delta_t) {

    MatrixXd Xsig_aug(n_aug_, n_sigma_);

    if (debug_)
        std::cout << "Predict " << delta_t << std::endl;
 
    GenerateAugmentedSigmaPoints(x_, P_, Xsig_aug);
    SigmaPointPrediction(delta_t, Xsig_aug, Xsig_pred_);
    PredictMeanAndCovariance(Xsig_pred_, x_, P_);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} mpage
 */
double UnscentedKalmanFilter::Update(const Measurement *m) {

  VectorXd z = m->measurements_;

  int n_z = z.rows();

  VectorXd z_pred(n_z);
  MatrixXd S(n_z,n_z);
  MatrixXd Zsig(n_z, n_sigma_);

  if (debug_)
      std::cout << "Update " << m->measurements_.transpose() << std::endl;

  m->PredictMeasurement(Xsig_pred_, weights_, Zsig, z_pred, S);
  UpdateState(z, z_pred, S, Xsig_pred_, Zsig, x_, P_);
  
  return Tools::CalculateNIS(z, z_pred, S);
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
    x_aug.head(n_x) = x;
    for (int i=n_x; i<n_aug_; i++)
	    x_aug(i) = 0.0;

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
  x(3) = Tools::NormalizeAngle(x(3));

  // predicted state covariance matrix
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    x_diff(3) = Tools::NormalizeAngle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //Eigen::EigenSolver<MatrixXd> es(P);
  //cout << "Eigenvalues of P pred:" << endl << es.eigenvalues() << endl;
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
           z_diff(1) = Tools::NormalizeAngle(z_diff(1));

       // state difference
       VectorXd x_diff = Xsig_pred.col(i) - x;
       x_diff(3) = Tools::NormalizeAngle(x_diff(3));

       Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   // residual
   VectorXd z_diff = z - z_pred;
   if (n_z == 3)
       z_diff(1) = Tools::NormalizeAngle(z_diff(1));

   // update state mean and covariance matrix
   x = x + K * z_diff;
   x(3) = Tools::NormalizeAngle(x(3));
   P = P - K*S*K.transpose();

   if (debug_ > 1) {
       std::cout << "Update done:" << endl;
       std::cout << "z:\n" << z.transpose() << std::endl;
       std::cout << "z_pred:\n" << z_pred.transpose() << std::endl;
       std::cout << "Xsig_pred:\n" << Xsig_pred << std::endl;
       std::cout << "Zsig:\n" << Zsig << std::endl;
       std::cout << "Tc:\n" << Tc << std::endl;
       std::cout << "S:\n" << S << std::endl;
       std::cout << "S inverse:\n" << S.inverse() << std::endl;
       std::cout << "K:\n" << K << std::endl;
       std::cout << "K transpose:\n" << K.transpose() << std::endl;
       std::cout << "P:\n" << P << std::endl;
       std::cout << "P inverse:\n" << P.inverse() << std::endl;
       std::cout << "x:\n" << x.transpose() << std::endl;
       Eigen::EigenSolver<MatrixXd> es(P);
       cout << "Eigenvalues of P:\n" << es.eigenvalues() << endl;
    }
}
