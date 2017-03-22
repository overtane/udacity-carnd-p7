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

    /**
     * Create measurement from input stream
     *
     * Method initialized also ground thruth if it is present in the stream
     *
     * @param iss Input string stream
     * @param sensors Sensor collection. There must exist a sensor instance that correspond to
     *                measurement. In other case, measurement is not instantiated 
     */
    Measurement *CreateMeasurement(std::istringstream &iss, const SensorContainer &sensors);
    /**
     * Create measurement for known attributes
     *
     * @param sensor Sensor instance pointer
     * @param timestamp Time of the measurement
     * @parma measurements Measurement values
     */
    Measurement *CreateMeasurement(const Sensor *sensor, long timestamp, VectorXd &measurement);

    static MeasurementFactory *GetInstance();
};

#endif
