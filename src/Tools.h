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
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

public:
  static const double PI2;

  /**
   * Normalize angle between [-pi, pi]
   */
  static double NormalizeAngle(double a);

  /** 
   * Check model consistency values 
   */
  static VectorXd CheckConsistency(const MeasurementContainer measurements,  const SensorContainer sensors);
  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const MeasurementContainer measurements);
  static VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
