#include "Tools.h"
#include <math.h>
#include <iostream>
#include <vector>

Tools::Tools() {}

Tools::~Tools() {}

const double Tools::PI2 = 2 * M_PI;
#define NORM_ANGLE_LOOP
//#define NORM_ANGLE_CEIL

double Tools::NormalizeAngle(double a) {
#if defined(NORM_ANGLE_LOOP)
    int n = 0;
    while (a > M_PI)  {a -= Tools::PI2; n++;}
    while (a < -M_PI) {a += Tools::PI2; n++;}
    if (n>0)
        std::cout << "normalize operations: " << n  << std::endl; 
    return a;
#elif defined(NORM_ANGLE_CEIL)
    return a - ceil((a-M_PI)/(Tools::PI2))*Tools::PI2;
#else
    return a;
#endif
}

double Tools::CalculateNIS(const VectorXd &z, const VectorXd &z_pred, const MatrixXd &S) {
  VectorXd z_diff = z - z_pred;
  return z_diff.transpose() * S.inverse() * z_diff;
}

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd> &estimations,
                              const std::vector<VectorXd> &ground_truth) {

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
	VectorXd e(1);
	e.fill(0.0);
        return e;
    }
    
    VectorXd rmse(4);
    rmse.fill(0.0);
    
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i) {
        
        VectorXd residual = estimations[i] - ground_truth[i];
        
        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    //calculate the mean
    rmse = rmse / estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    
    //check division by zero
    if (fabs(px)<0.001 && fabs(py)<0.001) {
        Hj << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
	std::cout << "Division by zero" << std::endl;
    } else {
        //compute the Jacobian matrix
        float dvdr2 = px*px + py*py;
        float dvdr1 = sqrt(dvdr2);
        float dvdr3 = dvdr1 * dvdr2;
        
        Hj << px/dvdr1,               py/dvdr1,               0,        0,
              -py/dvdr2,              px/dvdr2,               0,        0,
              py*(vx*py-vy*px)/dvdr3, px*(vy*px-vx*py)/dvdr3, px/dvdr1, py/dvdr1;
    }
    
    return Hj;


}
