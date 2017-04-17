#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "Eigen/Dense"
#include <string>
#include <map>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::string;

class UnscentedKalmanFilter;

class Sensor {

public:
    friend UnscentedKalmanFilter;

    /**
     * Set noise matrix from noise parameters
     */
    void SetR(const VectorXd& noise);
    /**
     * Get sensors noise matrix
     */
    const MatrixXd& GetR() const; 

    /**
     * Constructor
     */
    Sensor(string name, const VectorXd& noise, int df, int debug);
    /**
     * Destructor
     */
    virtual ~Sensor() {}
 
    ///* Sensor name
    const string name_;

    ///* Degrees of freedom, for consistency calculations
    const int df_;

    ///* Debug output flag
    const int debug_;

protected:

    /** 
     * Calculate 
     *
     * This method logically part of UKF update phase, a sensor type dependent part
     * Method is called from 
     *
     * @param Xsig_pred predicted sigma points
     * @param weights Weights matrix of the filter
     * @param Zsig Sigma points in measurement space (output)
     * @param z_pred Predicted measurment (output)
     * @param S Measurement covariance matrix (output)
     * @modified Use modified algorithm for covariance matrix
     */
    virtual void PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights_m, const VectorXd &weights_c, // inputs
		                    MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, bool modified) const = 0;         // outputs
   
    /**
     * Normalize angles in measurement vector z
     */
    virtual void NormalizeMeasurement(VectorXd &z) const = 0;

private:
    ///* Noise matrix
    MatrixXd R_;
};

typedef std::map<std::string, Sensor*> SensorContainer;


class LidarSensor : public Sensor {

public:

    void PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights_m, const VectorXd &weights_c, // inputs
		            MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &Si, bool modified) const;            // outputs
    void NormalizeMeasurement(VectorXd &) const;
 
    LidarSensor(string name, const VectorXd& noise, int debug);
    virtual ~LidarSensor() {}

};

class RadarSensor : public Sensor {

public:

    void PredictMeasurement(const MatrixXd &Xsig_pred, const VectorXd &weights_m, const VectorXd &weights_c, // inputs
		            MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S, bool modified) const;             // outputs
    void NormalizeMeasurement(VectorXd &) const;

    RadarSensor(string name, const VectorXd& noise, int debug);
    virtual ~RadarSensor() {}
};


#endif
