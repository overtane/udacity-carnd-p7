#ifndef TOOLS_H_
#define TOOLS_H_

#include "Eigen/Dense"
#include "Sensor.h"
#include "Measurement.h"
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;


class Tools {

private:
  /**
  * Private Constructor. Cannot instantiate outside the class
  *
  * This class do not have instantiable object. Only for static helper routines
  * (could use namespace for this instead)
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

public:
  ///* M_PI * 2;
  static const double PI2;

  /**
   * Normalize angle to be between [-pi, pi]
   */
  static double NormalizeAngle(double a, int debug);

  /** 
   * Check model consistency values 
   *
   * Routine goes through calculated NIS values and output a vector with and coefficient for 
   * each sensor type. The value of coeffient is the portion [0,1] of measurement above chi-squared(0.05) 
   * value (taken account the degrees of freedom of the sensor).
   */
  static VectorXd CheckConsistency(const MeasurementContainer measurements,  const SensorContainer sensors);

  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const MeasurementContainer measurements);

  /**
  * A helper method to calcula te RMSE. (EKF version)
  */
  static VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
