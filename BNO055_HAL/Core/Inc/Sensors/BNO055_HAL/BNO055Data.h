/*
 * bno055Data.h
 *
 *  Created on: Jul 24, 2023
 *      Author: ndanilo8
 */


#pragma once
#include "Sensors/SensorData.h"




struct BNO055Data : public AccelerometerData,
                     public GyroscopeData,
                     public MagnetometerData,
                     public TemperatureData
{
	BNO055Data()
	        : AccelerometerData{0, 0.0, 0.0, 0.0}, GyroscopeData{0, 0.0, 0.0, 0.0},
	          MagnetometerData{0, 0.0, 0.0, 0.0}, TemperatureData{0, 0.0}
	    {
	    }

	    static std::string header()
	    {
	        return "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,"
	               "gyro_y,gyro_z,mag_timestamp,mag_x,mag_y,mag_z,temp_timestamp,"
	               "temp\n";
	    }

	    void print(std::ostream& os) const
	    {
	        os << accel_timestamp << "," << accel_x << "," << accel_y << ","
	           << accel_z << "," << gyro_timestamp << "," << gyro_x << "," << gyro_y
	           << "," << gyro_z << "," << mag_timestamp << "," << mag_x << ","
	           << mag_y << "," << mag_z << "," << temp_timestamp << "," << temp
	           << "\n";
	    }

};
