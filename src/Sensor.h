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

    void SetR(const VectorXd& noise); 
    const MatrixXd& GetR() const; 

    virtual void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const = 0;
    virtual void NormalizeMeasurement(VectorXd &) const = 0;

    Sensor(string name, const VectorXd& noise, int df);
    virtual ~Sensor() {}

    string name_;
    int df_;

private:
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
