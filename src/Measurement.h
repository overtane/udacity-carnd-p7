#ifndef _MEASUREMENT_H_
#define _MEASUREMENT_H_

#include "Eigen/Dense"
#include <sstream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class UnscentedKalmanFilter;
class MeasurementFactory;

class Measurement {

public:
    friend UnscentedKalmanFilter;
    friend std::ostream& operator<<(std::ostream& o, const Measurement &m);
    static void SetR(const VectorXd&, MatrixXd&); 
    static const MatrixXd& GetR() const; 

    virtual VectorXd InitializeState(int n) const = 0;;
    virtual void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const = 0;
    virtual std::ostream& toString(std::ostream& o) const = 0;

//protected:
    Measurement(long timestamp, const VectorXd& measurements);

    VectorXd measurements_; 
    long timestamp_;
    double nis_;

    VectorXd estimate_;
    VectorXd ground_truth_;
};


class LidarMeasurement : public Measurement {

public:
    friend MeasurementFactory;

    static void SetR(const VectorXd&);

    VectorXd InitializeState(int n) const;
    void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const;
    const MatrixXd& GetR() const;
    virtual std::ostream& toString(std::ostream& o) const;

    virtual ~LidarMeasurement() {}

    static MatrixXd R_;
 
protected:
    LidarMeasurement(long timestamp, const VectorXd& measurements);

};

class RadarMeasurement : public Measurement {

public:
    friend MeasurementFactory;

    static void SetR(const VectorXd&);

    VectorXd InitializeState(int n) const;
    void PredictMeasurement(const MatrixXd &Xsig_pred, const MatrixXd &weights, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) const;
    const MatrixXd& GetR() const;
    virtual std::ostream& toString(std::ostream& o) const;

    virtual ~RadarMeasurement() {}

    static MatrixXd R_;

protected:
    RadarMeasurement(long timestamp, const VectorXd& measurements);
};


#endif
