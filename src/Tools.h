#ifndef TOOLS_H_
#define TOOLS_H_

#include "Eigen/Dense"
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
   * Calculate normalized innovation squared 
   */
  static double CalculateNIS(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S);
  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
