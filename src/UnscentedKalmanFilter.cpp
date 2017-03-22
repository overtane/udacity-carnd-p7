#include <iostream>
#include <math.h>
#include "Measurement.h"
#include "Sensor.h"
#include "UnscentedKalmanFilter.h"
#include "Tools.h"

using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UnscentedKalmanFilter::UnscentedKalmanFilter(double std_a, double std_yawdd, int pred_rate, int debug) :
    std_a_(std_a),
    std_yawdd_(std_yawdd),
    prediction_rate_(pred_rate),
    debug_(debug),	

    restart_(false),
    is_initialized_(false),

    n_x_(5),           // TODO parametrize this to get more general solution
    n_aug_(n_x_+2),
    lambda_(3-n_aug_), // TODO: how is lambda defined?
    n_sigma_(2*n_aug_+1),

    previous_measurement_(0),
    current_sensor_(0)
{
  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_); 

  // set weights
  weights_ = VectorXd(n_sigma_);
  weights_(0)= lambda_/(lambda_+n_aug_);
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights
    weights_(i) = .5/(n_aug_+lambda_);
  }
  
}

void UnscentedKalmanFilter::NormalizeState(VectorXd &x) const {
    x(3) = Tools::NormalizeAngle(x(3));
}

void UnscentedKalmanFilter::InitializeMeasurement(const Measurement *m) {
    
    // initialize the covariance matrix
    P_.fill(0.0); 
    for (int i=0; i<n_x_; ++i) {
    	    P_(i,i) = 0.1;
    }
    x_ = m->InitializeState(n_x_);

    // initial state (px,py) == (0,0) does not make sense
    if (fabs(x_(0))>0.001 && fabs(x_(1))>0.001)
        is_initialized_ = true;
    else if (debug_)
	std::cout << "Skip initial measurement: \n" << m->measurements_ << std::endl; 
    return;
}
/**
 * @param {MeasurementPackage} mp The latest measurement data of
 * either radar or laser.
 */
void UnscentedKalmanFilter::ProcessMeasurement(Measurement *m) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  current_sensor_ = m->sensor_;
  restart_ = false;

  if (!is_initialized_) {
      InitializeMeasurement(m);
      m->estimate_ = x_;
      previous_measurement_ = m;
      if ((debug_ >1) && is_initialized_) {
	std::cout << "Initial state x: \n" << x_ << std::endl;
	std::cout << "Initial covariance P: \n" << P_ << std::endl;
      }	
      return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Compute the time elapsed between the current and previous measurements, in seconds
    double dt = (m->timestamp_ - previous_measurement_->timestamp_) / 1000000.0;	
    double rdt = dt; // remaining delta time
    double pred_int = (prediction_rate_>0) ? 1.0/prediction_rate_ : rdt; // prediction_interval

    do {
  
        dt = (rdt > pred_int) ? pred_int : rdt;

        try {
             Prediction(dt);
        } catch (std::range_error e) {
            restart_ = true;
       	    // If convariance matrix is non positive definite (because of numerical instability?),
            // restart the filter using previous measurement as an initializer.
            InitializeMeasurement(previous_measurement_);
            // Redo prediction using the current measurement
            // We don't get exception this time, because intial P is positive definite.
            Prediction(dt);
        }

        rdt -= dt;

    } while (rdt > 0.0);


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // store results to measurements
  m->nis_ = Update(m);
  m->estimate_ = x_;

  previous_measurement_ = m;
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
double UnscentedKalmanFilter::Update(Measurement *m) {

  VectorXd z = m->measurements_;

  int n_z = z.rows();

  VectorXd z_pred(n_z);
  MatrixXd S(n_z,n_z);
  MatrixXd Zsig(n_z, n_sigma_);

  if (debug_)
      std::cout << "Update " << m->measurements_.transpose() << std::endl;

  current_sensor_->PredictMeasurement(Xsig_pred_, weights_, Zsig, z_pred, S);
  UpdateState(z, z_pred, S, Xsig_pred_, Zsig, x_, P_);
  
  return m->NIS(z_pred, S);
}

void UnscentedKalmanFilter::GenerateSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig) const {

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

bool UnscentedKalmanFilter::GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug) const {

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
        if (debug_) {
	    std::cout << "LLT failed!" << std::endl;
	    Eigen::EigenSolver<MatrixXd> es(P_aug);
	    cout << "P_aug:" << P_aug << endl;
	    cout << "Eigenvalues of P_aug:" << endl << es.eigenvalues() << endl;
	}
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

    if (debug_ > 2) {
        std::cout << "L:\n" << L << std::endl;
        std::cout << "Xsig_aug:\n" << Xsig_aug << std::endl;
    }
    return ret;
}

void UnscentedKalmanFilter::SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred) const {

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

void UnscentedKalmanFilter::PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) const {

  x.fill(0.0);
  P.fill(0.0);

  // predicted state mean
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points
      x = x + weights_(i) * Xsig_pred.col(i);
  }
  NormalizeState(x);

  // predicted state covariance matrix
  for (int i=0; i<n_sigma_; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    NormalizeState(x_diff);

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  if (debug_ > 2) {
    std::cout << "x pred:\n" << x << std::endl;
    std::cout << "P pred:\n" << P << std::endl;
    Eigen::EigenSolver<MatrixXd> es(P);
    std::cout << "Eigenvalues of P pred:\n" << es.eigenvalues() << std::endl;
  }
}
 
void UnscentedKalmanFilter::UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, 
		const MatrixXd &Xsig_pred, const MatrixXd &Zsig, VectorXd &x, MatrixXd &P) const {

   int n_z = z_pred.rows();	
   int n_x = x.rows();

   // calculate cross correlation matrix
   MatrixXd Tc(n_x, n_z);

   Tc.fill(0.0);

   for (int i=0; i<n_sigma_; i++) {  // 2n+1 sigma points

       // residual
       VectorXd z_diff = Zsig.col(i) - z_pred;
       current_sensor_->NormalizeMeasurement(z_diff);

       // state difference
       VectorXd x_diff = Xsig_pred.col(i) - x;
       NormalizeState(x_diff);

       Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // Kalman gain K;
   MatrixXd K = Tc * S.inverse();

   // residual
   VectorXd z_diff = z - z_pred;
   current_sensor_->NormalizeMeasurement(z_diff);

   // update state mean and covariance matrix
   x = x + K * z_diff;
   NormalizeState(x);
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
