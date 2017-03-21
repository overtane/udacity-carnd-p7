#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

#include "Eigen/Dense"
#include <sstream>
#include <vector>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

class UnscentedKalmanFilter;
class MeasurementFactory;
class Sensor;
class Tools;

class Measurement {

public:

    friend UnscentedKalmanFilter;
    friend MeasurementFactory;
    friend Tools;
    friend std::ostream& operator<<(std::ostream& o, const Measurement &m);

    virtual VectorXd InitializeState(int n) const = 0;

    virtual std::ostream& toString(std::ostream& o) const;
    double NIS(const VectorXd &z_pred, const MatrixXd &S);

protected:
    Measurement(const Sensor *sensor, long timestamp, const VectorXd& measurements);

    const Sensor *sensor_;
    const VectorXd measurements_; 
    const long timestamp_;

    double nis_;

    VectorXd estimate_;
    VectorXd ground_truth_;
};

typedef  vector<Measurement*> MeasurementContainer;

class LidarMeasurement : public Measurement {

public:
    friend MeasurementFactory;

    VectorXd InitializeState(int n) const;
    virtual std::ostream& toString(std::ostream& o) const;

    virtual ~LidarMeasurement() {}
 
protected:
    LidarMeasurement(const Sensor *sensor, long timestamp, const VectorXd& measurements);

};

class RadarMeasurement : public Measurement {

public:
    friend MeasurementFactory;

    VectorXd InitializeState(int n) const;
    virtual std::ostream& toString(std::ostream& o) const;

    virtual ~RadarMeasurement() {}

protected:
    RadarMeasurement(const Sensor *sensor, long timestamp, const VectorXd& measurements);
};


#endif
