#ifndef _MEASUREMENT_FACTORY_H_
#define _MEASUREMENT_FACTORY_H_

#include "Eigen/Dense"
#include "Sensor.h"
#include <sstream>


using Eigen::VectorXd;
using std::string;

class Measurement;

class MeasurementFactory
{
private:

    static MeasurementFactory *instance_;

    // private default constructor, construction only possible inside class method 
    MeasurementFactory() {};
    // private copy constructor, no implemenation. prevents coping of the instance 
    MeasurementFactory(const MeasurementFactory&);
    // private assignment operator, no implementation, prevent assignment of the instance
    MeasurementFactory& operator =(const MeasurementFactory&);

public:
    virtual ~MeasurementFactory() {};

    Measurement *CreateMeasurement(std::istringstream &, const SensorContainer &sensors);
    Measurement *CreateMeasurement(const Sensor *sensor, long timestamp, VectorXd &measurement);

    static MeasurementFactory *GetInstance();
};

#endif
