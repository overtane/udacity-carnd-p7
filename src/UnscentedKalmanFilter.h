#ifndef UNSCENTED_KALMAN_FILTER_H_
#define UNSCENTED_KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Sensor;
class Measurement;

class UnscentedKalmanFilter {

public:

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* the minimum rate predictions are run.
  double prediction_rate_;

  ///* Debugging level, 1: print intermediate phases, 2: print intermediate matrices, 3: print even more matrices
  int debug_;

  ///* Filter restarted
  bool restart_;

  ///* Previous measurement
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Sigma points dimentsion
  int n_sigma_;

  Measurement *previous_measurement_;

  ///* Sensor instance of the current measurement
  const Sensor *current_sensor_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  /**
   * Constructor
   * @param std_a Process noise standard deviation longitudinal acceleration in m/s^2
   * @param std_yawdd Process noise standard deviation yaw acceleration in rad/s^2
   * @param pred_rate Minimum rate to run predictions
   * @param debug Debugging level
   */
  UnscentedKalmanFilter(double std_a, double std_yawdd, int pred_rate, int debug);

  /**
   * Destructor
   */
  virtual ~UnscentedKalmanFilter() {}

  /**
   * ProcessMeasurement
   * @param m  The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(Measurement *m);


private:

  /**
   * Default constructor is private preventing using it outside the class
   */
  UnscentedKalmanFilter() {}

  /**
   * Initialize state vector and covariance matrix
   *
   * Method set member variable is_initialized_ to true if measurement is valid.
   *
   * @param m Measurement to be used to initiaze state vector
   */
  void InitializeMeasurement(const Measurement *m);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix delta_t from now
   *
   * @param delta_t
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param m The measurement at k+1
   */
  double Update(Measurement *m);

  /**
   * Generate sigma points for prediction step
   *
   * Note: just for reference, not used in this filter 
   *
   * @param x Current state vector
   * @param P Current covariance matrix
   * @param Xsig_out Generated sigma points (output)
   */
  void GenerateSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_out) const;
     
  /**
   * Generate augmented sigma points for prediction step
   *
   * @param x Current state vector
   * @param P Current covariance matrix
   * @param Xsig_out Generated augmented sigma points matrix (output)
   */ 
  bool GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug) const;
        
  /**
   * Prediction step for sigma points
   *
   * @param delta_t Time delta
   * @param Xsig_aug Augmented sigma points matrix (output)
   * @param Xsig_pred Predicted sigma points matrix (output)
   */
  void SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred) const;
  
  /**
   * Generate predicted state and covariance matrix from predicted sigma points
   *
   * @param Xsig_pred Predicted sigma points
   * @param x predicted state vector (output)
   * @param P predicted covariance matrix (output)
   */
  void PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P) const;
	   
  /**
   * Update state from a measurement
   *
   * @param z Current measurement
   * @param z_pred Predicted measurement
   * @param S sensor covariance matrix
   * @param Zsig Measurement sigma points
   * @param Xsig_pre Predicted sigma points
   * @param x state vector (output)
   * @parma P covariance matrix (output)
   *
   */
  void UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, 
		    const MatrixXd &Zsig, const MatrixXd &Xsig_pred, 
		     VectorXd &x, MatrixXd &P) const;

  /** 
   * Normalize angles in state vector
   *
   * All angles are expressed in radians. Set the values between [-pi, pi]
   *
   * @param x A state vector
   */
  void NormalizeState(VectorXd &x) const;
};

#endif /* UNSCENTED_KALMAN_FILTER_H_ */
