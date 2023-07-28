/*
 * Sensor.h
 *
 */

#ifndef INC_SENSORS_SENSOR_H_
#define INC_SENSORS_SENSOR_H_

#include "SensorData.h"

/**
 * @brief Base abstract class for sensor drivers.
 */
class AbstractSensor
{
protected:
    SensorErrors last_error = SensorErrors::NO_ERRORS;

public:
    virtual ~AbstractSensor() {}

    /**
     * @brief Initialize the sensor.
     * @return boolean value indicating whether the operation succeded or not
     */
    virtual bool init() = 0;

    /**
     * @brief Check if the sensor is working.
     * @return boolean indicating whether the sensor is correctly working or not
     */
    virtual bool selfTest() = 0;

    /**
     * @brief Sample the sensor.
     */
    virtual void sample() = 0;

    /**
     * @brief Get last error for debugging purposes. Avoid silent fails.
     * @return the last error recorded by this sensor
     */
    SensorErrors getLastError() { return last_error; };
};

/**
 * @brief Base sensor class with has to be extended by any sensor driver.
 *
 * A sensor driver can define a custom data structure extending any
 * combination of base sensors data structures, defined in `SensorData.h`.
 */
template <typename Data>
class Sensor : public virtual AbstractSensor
{
protected:
    Data last_sample;

    /**
     * @brief Read a data sample from the sensor.
     *        In case of errors, the method should return the last
     *        available correct sample.
     * @return sensor data sample
     */
    virtual Data sampleImpl() = 0;

public:
    virtual ~Sensor() {}

    void sample() override { last_sample = sampleImpl(); }

    /**
     * @return last available sample from this sensor
     */
    virtual const Data& getLastSample() { return last_sample; }
};



#endif /* INC_SENSORS_SENSOR_H_ */
