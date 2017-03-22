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
    Sensor(string name, const VectorXd& noise, int df);
    /**
     * Destructor
     */
    virtual ~Sensor() {}
 
    ///* Sensor name
    string name_;
    ///* Degrees of freedom, for consistency calculations
    int df_;

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
     */
    virtual void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const = 0;
   
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

    void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const;
    void NormalizeMeasurement(VectorXd &) const;
 
    LidarSensor(string name, const VectorXd& noise);
    virtual ~LidarSensor() {}

};

class RadarSensor : public Sensor {

public:

    void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const;
    void NormalizeMeasurement(VectorXd &) const;

    RadarSensor(string name, const VectorXd& noise);
    virtual ~RadarSensor() {}
};


#endif
