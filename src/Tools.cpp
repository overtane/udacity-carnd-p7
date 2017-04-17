#include "Sensor.h"
#include "Measurement.h"
#include "Tools.h"
#include <math.h>
#include <iostream>
#include <vector>

Tools::Tools() {}

Tools::~Tools() {}

const double Tools::PI2 = 2 * M_PI;
#define NORM_ANGLE_LOOP
//#define NORM_ANGLE_CEIL

double Tools::NormalizeAngle(double a, int debug) {
#if defined(NORM_ANGLE_LOOP)
    int n = 0;
    while (a > M_PI)  {a -= Tools::PI2; n++;}
    while (a < -M_PI) {a += Tools::PI2; n++;}
    if (n>0 && debug)
        std::cout << "normalize operations: " << n  << std::endl; 
    return a;
#elif defined(NORM_ANGLE_CEIL)
    return a - ceil((a-M_PI)/(Tools::PI2))*Tools::PI2;
#else
    return a;
#endif
}

// chi squared 5% limit per degrees of freedom
const static double ChiSquared5[] = {3.841, 5.991, 7.815, 9.488, 11.070}; 

VectorXd Tools::CheckConsistency(const MeasurementContainer measurements,  const SensorContainer sensors) {
    
    VectorXd result(sensors.size());
   
    struct Totals {
        int n;
        int tot;
    };

    std::map<string, Totals *> totals;

    // Add all sensors to total map
    for (SensorContainer::const_iterator it=sensors.begin(); it!=sensors.end(); ++it) {
	Totals *t = new Totals;
	t->n = t->tot = 0;
	string name = it->first;
        totals[name] = t;
    }

    // Handle measurements
    // Count total number of measurements per sensor, and number of estimates above consistency limit
    for (MeasurementContainer::const_iterator it=measurements.begin(); it!=measurements.end(); ++it) {
	const Measurement *m = *it;
	const Sensor *s = m->sensor_;
        Totals *t = totals[s->name_];
	t->tot++;
	if (m->nis_ > ChiSquared5[s->df_-1]) { // increment based on degrees of freedom per sensor.
            //std::cout << m->nis_ << std::endl;
            t->n++;
	}
    }

    // Calculate consistency value
    std::map<string, Totals *>::iterator it;
    int i;
    for (it=totals.begin(), i=0; it!=totals.end(); it++,i++) {
        Totals *t = it->second;
        result(i) = (t->n*1.0)/t->tot;
	delete it->second;
    }
    
    return result;
}

VectorXd Tools::CalculateRMSE(const MeasurementContainer measurements) {

    VectorXd rmse(7); // [px, py, vx, vy, v, psi, psi dot]T
    rmse.fill(0.0);
    
    //accumulate squared residuals
    for (MeasurementContainer::const_iterator it=measurements.begin(); it!=measurements.end(); ++it) {

        VectorXd esc(7);                      // calculated estimates
        VectorXd eso = (*it)->estimate_;      // original estimate
        VectorXd gtc(7);                      // calculated ground truth
        VectorXd gto = (*it)->ground_truth_;  // original ground truth

        gtc(0) = gto(0); 
	gtc(1) = gto(1);
        gtc(2) = gto(2);
	gtc(3) = gto(3);
	double s = (gtc(3)<0)?-1.0:1.0; // angle sign
        gtc(4) = sqrt(gto(2)*gto(2)+gto(3)*gto(3));
	gtc(5) = (fabs(gtc(4))<0.0001) ? M_PI/2 : acos(gto(2)/gtc(4));
	gtc(5) *= s;
	gtc(6) = 0;

        esc(0) = eso(0);
	esc(1) = eso(1);
	esc(2) = eso(2) * cos(eso(3));
	esc(3) = eso(2) * sin(eso(3));
	esc(4) = eso(2);
	esc(5) = eso(3);
	esc(6) = eso(4);

        VectorXd residual = esc- gtc;
        
        // coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    
    // calculate the mean
    rmse = rmse / measurements.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse.head(4);  // for now, return only px, py, vx, vy
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
