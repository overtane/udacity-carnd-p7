#ifndef UNSCENTED_KALMAN_FILTER_H_
#define UNSCENTED_KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "Measurement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Sensor;

class UnscentedKalmanFilter {

public:

  int debug_;

  bool restart_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  int n_sigma_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  Measurement *previous_measurement_;
  const Sensor *current_sensor_;

  /**
   * Constructor
   */
  UnscentedKalmanFilter(double, double, int);

  /**
   * Destructor
   */
  virtual ~UnscentedKalmanFilter() {}

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   * @param gt_package The ground truth of the state x at measurement time
   */
  void ProcessMeasurement(Measurement *m);


private:

  UnscentedKalmanFilter() {}

  void InitializeMeasurement(const Measurement *m);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  double Update(Measurement *m);

    void GenerateSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_out);
      
    bool GenerateAugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, MatrixXd &Xsig_aug);
        
    void SigmaPointPrediction(double delta_t, const MatrixXd &Xsig_aug, MatrixXd &Xsig_pred);
	  
    void PredictMeanAndCovariance(const MatrixXd &Xsig_pred, VectorXd &x, MatrixXd &P);
	   
    void UpdateState(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S, 
		     const MatrixXd &Zsig, const MatrixXd &Xsig_pred, 
		     VectorXd &x, MatrixXd &P);

    void NormalizeState(VectorXd &x);
};

#endif /* UNSCENTED_KALMAN_FILTER_H_ */
