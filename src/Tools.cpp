#include <iostream>
#include "Tools.h"

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
        return rmse;
    }
    
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
    
    //TODO: YOUR CODE HERE
    
    //check division by zero
    if (fabs(px)<0.001 && fabs(py)<0.001) {
        Hj << 0, 0, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
        cout << "Division by zero" << endl;
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
