/*
 * bno055_hal.cpp
 *
 *  Created on: Jul 24, 2023
 *      Author: ndanilo8
 */

#include "BNO055_HAL.h"
// #include "TimestampTimer.h"

BNO055_HAL::BNO055_HAL(I2C_HandleTypeDef &i2cHandle) :
		_i2cHandle(i2cHandle) {

}

bool BNO055_HAL::init() {
	//check if sensor already initialized
	if (initialized) {

		// log "Already Initialized"
		last_error = SensorErrors::ALREADY_INIT;
		return false;
	}

	// read chip id
	_deviceInfo.chipID =

	//set oage to 0

			//reset

			//self test

			//set ext crystal use

			// normal power mode

			// unit config

			//Interrupt Config

			// set sensor configs

			// axis Mapping

			// set opr mode to AMG

			initialized = true;

	return true;

}

BNO055Data BNO055_HAL::sampleImpl() {

	if (!initialized) {

		// log error "invoked sampleImpl() but sensor was unitialized
		last_error = SensorErrors::NOT_INIT;
		return last_sample;
	}
	// Ensure that the sensor is in the correct page
	if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
		setPage(BNO055_PAGE_ID::PAGE_ZERO);
	}

	// if to check sys status for new data
	// read registers without Interrupts (1st phase) just to check if we can get data from sensor
	// if that works then update code using interrupts
	/*
	 if (status != newdatafromsensors) {

	 last_error = SensorErrors::NO_NEW_DATA;
	 return last_sample;
	 }
	 */

	// Reset any error
	last_error = SensorErrors::NO_ERRORS;

	// Create a new BNO055Data object to store the sampled data
	BNO055Data newData { };

	// Get the current timestamp
	uint64_t timestamp =HAL_GetTick();// TimestampTimer::getTimestamp();
	newData.accel_timestamp = timestamp;
	newData.temp_timestamp = timestamp;
	newData.gyro_timestamp = timestamp;
	newData.mag_timestamp = timestamp;

	// Buffer to store the raw sensor data
	uint8_t buffer[18];

	// Read raw accelerometer, gyroscope, and magnetometer data (interrupts are not used yet)
	if (readLen(BNO055_Registers::ACCEL_DATA_X_LSB_ADDR, buffer, 18)
			&& _deviceInfo.pageID == BNO055_PAGE_ID::PAGE_ZERO) {

		// Raw Accelerometer Data
		newData.accel_x = (uint16_t) ((buffer[1] << 8) | buffer[0])
				/ _sensorScales.accel;
		newData.accel_y = (uint16_t) ((buffer[3] << 8) | buffer[2])
				/ _sensorScales.accel;
		newData.accel_z = (uint16_t) ((buffer[5] << 8) | buffer[4])
				/ _sensorScales.accel;

		// Raw Magnetometer Data
		newData.mag_x = (uint16_t) ((buffer[7] << 8) | buffer[6])
				/ _sensorScales.mag;
		newData.mag_y = (uint16_t) ((buffer[9] << 8) | buffer[8])
				/ _sensorScales.mag;
		newData.mag_z = (uint16_t) ((buffer[11] << 8) | buffer[10])
				/ _sensorScales.mag;

		// Raw Temperature Data
		newData.gyro_x = (uint16_t) ((buffer[13] << 8) | buffer[12])
				/ _sensorScales.gyro;
		newData.gyro_y = (uint16_t) ((buffer[15] << 8) | buffer[14])
				/ _sensorScales.gyro;
		newData.gyro_z = (uint16_t) ((buffer[17] << 8) | buffer[16])
				/ _sensorScales.gyro;

		// Raw Temperature Data
		uint8_t rawTemperature;
		read8(BNO055_Registers::TEMP_ADDR, &rawTemperature);
		newData.temp = BNO055_GET_BITSLICE(rawTemperature, BNO055_TEMP)
				/ _sensorScales.temp;

		// Save the newly sampled data as the last sample for future reference
		last_sample = newData;

		// Return the newly sampled data
		return newData;

	} else {
		// Error handling: Unable to read data from the sensor or incorrect page ID
		last_error = SensorErrors::NO_NEW_DATA;
		// Return the last known valid data
		// (keeps old data for consistency in case of errors)

		return last_sample;
	}
	return last_sample;
}

bool BNO055_HAL::selfTest() {

	// Check if the current page is PAGE_ZERO
	if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
		setPage(BNO055_PAGE_ID::PAGE_ZERO);
	}
	// Check if the current Operation Mode is OPERATION_MODE_CONFIG
	if (_deviceInfo.oprMode != BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {
		setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG);
	}

	// Proceed only if the current page is PAGE_ZERO
	if (_deviceInfo.pageID == BNO055_PAGE_ID::PAGE_ZERO
			&& _deviceInfo.oprMode == BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {

		// Trigger SelfTest
		uint8_t triggerSelfTest = 0; // variable to set LSB = 0
		triggerSelfTest = BNO055_SET_BITSLICE(triggerSelfTest, BNO055_SELFTEST,
				0); // Set bit 0 to 0 for self-test
		write8(BNO055_Registers::SYS_TRIGGER_ADDR, triggerSelfTest); // Write the value to the SYS_TRIGGER register

		// this lines check the sys status reg to see if the self test is being run or not
		//uint8_t status;
		// (if status = 4 its Executing selftest, as per datasheet 4.3.58 SYS_STATUS 0x39
		//read8(BNO055_Registers::SYS_STAT_ADDR, &status);
		//status = BNO055_GET_BITSLICE(status, BNO055_SYS_STAT_CODE);

		HAL_Delay(400); // as per datasheet 3.9.2 Built-In Self-Test (BIST)

		// Read SelfTest result
		uint8_t tmp; // variable to hold selftest result
		if (read8(BNO055_Registers::SELFTEST_RESULT_ADDR, &tmp)
				== HAL_StatusTypeDef::HAL_OK) {
			// Extract self-test result for each component
			_selfTestResult.mcuState = BNO055_GET_BITSLICE(tmp,
					BNO055_SELFTEST_MCU);
			_selfTestResult.gyroState = BNO055_GET_BITSLICE(tmp,
					BNO055_SELFTEST_GYRO);
			_selfTestResult.magState = BNO055_GET_BITSLICE(tmp,
					BNO055_SELFTEST_MAG);
			_selfTestResult.accelState = BNO055_GET_BITSLICE(tmp,
					BNO055_SELFTEST_ACCEL);

			// Check if all self-test results are successful (all values are 1)
			if (_selfTestResult.mcuState != 1 || _selfTestResult.gyroState != 1
					|| _selfTestResult.magState != 1
					|| _selfTestResult.accelState != 1) {
				// At least one value is 0, indicating self-test failure
				last_error = SensorErrors::SELF_TEST_FAIL;
				return false;

			} else {
				// All self-test results are successful (all values are 1)
				last_error = SensorErrors::NO_ERRORS;
				return true;
			}
		}
	}
	// If the function reaches this point, it means there was an error
	// setting the page or Operation mode or reading the self-test result
	last_error = SensorErrors::SELF_TEST_FAIL;
	return false;
}

bool BNO055_HAL::setPage(BNO055_PAGE_ID page) {

	// check if the requested page is different. if it is then change
	if (_deviceInfo.pageID != page) {
// dont know why the "page" enum with an underlying type "uint8_t" doesnt work.. so for now ill static cast it
		if (write8(BNO055_Registers::PAGE_ID_ADDR, page)
				== HAL_StatusTypeDef::HAL_OK) {
			_deviceInfo.pageID = page;
			return true;
		}
	}
	return false;

}

bool BNO055_HAL::setOperationMode(BNO055_OPR_MODE mode) {

	if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
		setPage(BNO055_PAGE_ID::PAGE_ZERO);
	}
	if (_deviceInfo.oprMode != mode
			&& _deviceInfo.pageID == BNO055_PAGE_ID::PAGE_ZERO) {


		if (write8(BNO055_Registers::OPR_MODE_ADDR, mode)
				== HAL_StatusTypeDef::HAL_OK) {

			_deviceInfo.oprMode = mode;

			// p22 table 3-6: Operation moder switching time
			if (mode == BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {
				HAL_Delay(19);
			} else {
				HAL_Delay(7);
			}

			return true;
		}
	}

	return false;
}

bool BNO055_HAL::setExternalCrystralUse(bool state) {

	if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
		setPage(BNO055_PAGE_ID::PAGE_ZERO);
	}
	uint8_t tmp = 0;
	read8(BNO055_Registers::SYS_TRIGGER_ADDR, &tmp);
	if (state == true) {
		tmp |= 0x80;  // Set the MSB (most significant bit) of tmp to 1
	} else {
		tmp &= 0x0;  // Clear the MSB of tmp
	}

	write8(BNO055_Registers::SYS_TRIGGER_ADDR, tmp);
	HAL_Delay(700);
	return true;

}

/**
 * @brief Set the power mode of the accelerometer in the BNO055 sensor.
 * @param setPowerMode The desired power mode for the accelerometer.
 * @return true if the power mode was successfully set, false otherwise.
 */
bool BNO055_HAL::setAccelPWRMode(BNO055_ACCEL_PWR_MODE setPowerMode) {

	// Set the operation mode to CONFIG mode to configure the device.
	if (setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG)) {

		// Switch to PAGE_ONE to access the ACCEL_CONFIG_ADDR register.
		setPage(BNO055_PAGE_ID::PAGE_ONE);

		// Read the current ACCEL_CONFIG_ADDR register value.
		uint8_t accelPWRMode = 0;

		// Update the power mode in the accelPWRMode variable using the BNO055_SET_BITSLICE macro.
		accelPWRMode = BNO055_SET_BITSLICE(accelPWRMode,
				BNO055_ACCEL_POWER_MODE, setPowerMode);

		// Write the updated accelPWRMode value back to the ACCEL_CONFIG_ADDR register.
		if (write8(BNO055_Registers::ACCEL_CONFIG_ADDR, accelPWRMode)
				== HAL_StatusTypeDef::HAL_OK) {

			// If the write operation was successful, return true
			return true;
		}

	}
	// If any of the previous steps failed, return false to indicate that the power mode was not set.
	return false;
}

/*
 * LOW LEVEL FUNCTIONS
 * */
HAL_StatusTypeDef BNO055_HAL::write8(uint8_t regAddr, uint8_t data) {
	// Start the I2C memory write operation using interrupts
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&_i2cHandle,
	BNO055_I2C_ADDR, regAddr,
	I2C_MEMADD_SIZE_8BIT, &data, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		last_error = SensorErrors::BUS_FAULT;
		return status;
	}

	// Return HAL_OK to indicate successful data read.
	return HAL_OK;

}

HAL_StatusTypeDef BNO055_HAL::read8(uint8_t regAddr, uint8_t *data) {
	// A read sequence consists of a one-byte I²C write phase followed by the I²C read phase.
	// The I²C write phase	addresses the slave and sends the register address to be read.
	// Start the I2C memory write operation to send the register address using interrupts
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&_i2cHandle,
	BNO055_I2C_ADDR, &regAddr, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them for the write phase
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		last_error = SensorErrors::BUS_FAULT;
		return status;
	}

	// Start the I2C memory read operation using interrupts
	status = HAL_I2C_Mem_Read_IT(&_i2cHandle, BNO055_I2C_ADDR, regAddr,
	I2C_MEMADD_SIZE_8BIT, data, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them for the read phase
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		last_error = SensorErrors::BUS_FAULT;
		return status;
	}

	// Return HAL_OK to indicate successful data read.
	return HAL_OK;
}

HAL_StatusTypeDef BNO055_HAL::readLen(uint8_t regAddr, uint8_t *data,
		uint8_t length) {
	// A read sequence consists of a one-byte I²C write phase followed by the I²C read phase. (p102 BNO055 Datasheet)
	// The I²C write phase	addresses the slave and sends the register address to be read.

	// Start the I2C memory write operation to send the register address using interrupts
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&_i2cHandle,
	BNO055_I2C_ADDR, &regAddr, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them for the write phase
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		last_error = SensorErrors::BUS_FAULT;
		return status;
	}

	// Start the I2C memory read operation using interrupts
	status = HAL_I2C_Mem_Read_IT(&_i2cHandle, BNO055_I2C_ADDR, regAddr,
	I2C_MEMADD_SIZE_8BIT, data, length);

	// Check for I2C errors and handle them for the read phase
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		last_error = SensorErrors::BUS_FAULT;
		return status;
	}

	// Return HAL_OK to indicate successful data read.
	return HAL_OK;
}
