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
    /**
     * output operator
     */
    friend std::ostream& operator<<(std::ostream& o, const Measurement &m);

    /**
     * UKF state vector initializer
     *
     * This is called for measurement after filter start or restart
     *
     * @param n Filter's state vector size 
     */
    virtual VectorXd InitializeState(int n) const = 0;

    /**
     * Stringify measurement object for output file
     */
    virtual std::ostream& toString(std::ostream& o) const;

    /**
     * Normalized innovation squared calculation
     *
     * @param z_pred predicted measurement
     * @parma S covariance matrix
     */
    double NIS(const VectorXd &z_pred, const MatrixXd &S);

protected:
    /**
     * Constructor, protected for factory access only
     *
     * @param sensor Sensor instance where measurement originates
     * @param timestamp Time of the measurement
     * @param measurements Measurement vector
     */
    Measurement(const Sensor *sensor, long timestamp, const VectorXd& measurements);

    ///* sensor instance
    const Sensor *sensor_;
    //* timestamp
    const long timestamp_;
    ///* measurement values
    const VectorXd measurements_; 

    ///* calculated NIS value for the measurement
    double nis_;

    ///* estimate from the UKF
    VectorXd estimate_;
    ///* ground truth from the input
    VectorXd ground_truth_;
};


///* Measurement container class
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
