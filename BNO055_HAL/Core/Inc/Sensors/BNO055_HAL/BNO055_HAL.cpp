/*
 * bno055_hal.cpp
 *
 *  Created on: Jul 24, 2023
 *      Author: ndanilo8
 */

// in have to organize the return types of the functions
#include "BNO055_HAL.h"
// #include "TimestampTimer.h"

BNO055_HAL::BNO055_HAL(I2C_HandleTypeDef &i2cHandle) :
		_i2cHandle(i2cHandle) {

}

bool BNO055_HAL::init() {

	HAL_StatusTypeDef status;

	//check if sensor already initialized
	if (initialized) {

		// log "Already Initialized"
		last_error = SensorErrors::ALREADY_INIT;
		return false;
	}
	//set page to 0
	setPage(BNO055_PAGE_ID::PAGE_ZERO);

	// read chip id
	status = read8(BNO055_Registers::CHIP_ID_ADDR, &_deviceInfo.chipID);

	if (_deviceInfo.chipID == BNO055_CHIP_ID
			&& status == HAL_StatusTypeDef::HAL_OK) {
		last_error = SensorErrors::NO_ERRORS;
	} else {
		last_error = SensorErrors::INVALID_WHOAMI;
		initialized = false;
		return false;
	}

	//reset
	uint8_t resetflag = 0;

	resetflag = BNO055_SET_BITSLICE(resetflag, BNO055_SYS_RST,
			BNO055_BIT_ENABLE);
	status = write8(BNO055_Registers::SYS_TRIGGER_ADDR, resetflag);

	if (status == HAL_StatusTypeDef::HAL_OK) {
		last_error = SensorErrors::INIT_FAIL;
		initialized = false;
		return false;
	}

	//self test
	bool selfTestResult = selfTest();
	if (!selfTestResult) {
		last_error = SensorErrors::SELF_TEST_FAIL;
		initialized = false;
		return false;
	}
	//set ext crystal use
	setExternalCrystalUse(true);
	// normal power mode
	setPWRMode(BNO055_PWR_MODE::POWER_MODE_NORMAL);
// unit config
	unitSelect(BNO055_ACCEL_UNITS::ACCEL_UNIT_MSQ,
			BNO055_GYRO_UNITS::GYRO_UNIT_RPS,
			BNO055_TEMP_UNITS::TEMP_UNIT_CELSIUS);

//Interrupt Config

// set sensor configs
	setAccelConfig(BNO055_ACCEL_PWR_MODE::ACCEL_NORMAL,
			BNO055_ACCEL_BW::ACCEL_BW_125HZ,
			BNO055_ACCEL_RANGE::ACCEL_RANGE_8G);

	setMagConfig(BNO055_MAG_PWR_MODE::MAG_POWER_MODE_NORMAL,
			BNO055_MAG_OPR_MODE::MAG_OPERATION_MODE_HIGH_ACCURACY,
			BNO055_MAG_OUTPUT_RATE::MAG_DATA_OUTRATE_30HZ);

	setGyroConfig(BNO055_GYRO_PWR_MODE::GYRO_POWER_MODE_NORMAL,
			BNO055_GYRO_BW::GYRO_BW_230HZ,
			BNO055_GYRO_RANGE::GYRO_RANGE_2000DPS);

// axis Mapping

// set opr mode to AMG
	setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_AMG);

	initialized = true;
	return true;

}

BNO055Data BNO055_HAL::sampleImpl() {
	uint8_t status;

	// Create a new BNO055Data object to store the sampled data
	BNO055Data newData { };

	if (!initialized) {

		// log error "invoked sampleImpl() but sensor was unitialized
		last_error = SensorErrors::NOT_INIT;
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

// Get the current timestamp
	uint64_t timestamp = HAL_GetTick();	// TimestampTimer::getTimestamp();
	newData.accel_timestamp = timestamp;
	newData.temp_timestamp = timestamp;
	newData.gyro_timestamp = timestamp;
	newData.mag_timestamp = timestamp;

// Buffer to store the raw sensor data
	uint8_t buffer[18];

// Read raw accelerometer, gyroscope, and magnetometer data (interrupts are not used yet)
	status = readLen(BNO055_Registers::ACCEL_DATA_X_LSB_ADDR, buffer, 18);
	/*
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
	 */

	// Raw Accelerometer Data
	newData.accel_x = CONVERT_RAW_DATA(buffer, 0, _sensorScales.accel);
	newData.accel_y = CONVERT_RAW_DATA(buffer, 2, _sensorScales.accel);
	newData.accel_z = CONVERT_RAW_DATA(buffer, 4, _sensorScales.accel);

	// Raw Magnetometer Data
	newData.mag_x = CONVERT_RAW_DATA(buffer, 6, _sensorScales.mag);
	newData.mag_y = CONVERT_RAW_DATA(buffer, 8, _sensorScales.mag);
	newData.mag_z = CONVERT_RAW_DATA(buffer, 10, _sensorScales.mag);

	// Raw Temperature Data
	newData.gyro_x = CONVERT_RAW_DATA(buffer, 12, _sensorScales.gyro);
	newData.gyro_y = CONVERT_RAW_DATA(buffer, 14, _sensorScales.gyro);
	newData.gyro_z = CONVERT_RAW_DATA(buffer, 16, _sensorScales.gyro);

	// Raw Temperature Data
	uint8_t rawTemperature;
	status = read8(BNO055_Registers::TEMP_ADDR, &rawTemperature);
	newData.temp = BNO055_GET_BITSLICE(rawTemperature, BNO055_TEMP)
			/ _sensorScales.temp;


// Return the newly sampled data
return newData;

}

bool BNO055_HAL::selfTest() {
uint8_t status;
// Check if the current page is PAGE_ZERO
if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
	setPage(BNO055_PAGE_ID::PAGE_ZERO);
}
// Check if the current Operation Mode is OPERATION_MODE_CONFIG
if (_deviceInfo.oprMode != BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {
	setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG);
}

// Proceed only if the current page is PAGE_ZERO and mode is CONFIG
if (_deviceInfo.pageID == BNO055_PAGE_ID::PAGE_ZERO
		&& _deviceInfo.oprMode == BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {

	// Trigger SelfTest
	uint8_t triggerSelfTest = 0; // variable to set LSB = 0
	triggerSelfTest = BNO055_SET_BITSLICE(triggerSelfTest, BNO055_SELFTEST,
			BNO055_BIT_DISABLE); // Set bit 0 to 0 for self-test
	status = write8(BNO055_Registers::SYS_TRIGGER_ADDR, triggerSelfTest); // Write the value to the SYS_TRIGGER register

	// this lines check the sys status reg to see if the self test is being run or not
	//uint8_t status;
	// (if status = 4 its Executing selftest, as per datasheet 4.3.58 SYS_STATUS 0x39
	//read8(BNO055_Registers::SYS_STAT_ADDR, &status);
	//status = BNO055_GET_BITSLICE(status, BNO055_SYS_STAT_CODE);
	if (status == HAL_StatusTypeDef::HAL_OK) {
		HAL_Delay(400); // as per datasheet 3.9.2 Built-In Self-Test (BIST)

		// Read SelfTest result
		uint8_t tmp; // variable to hold selftest result
		status = read8(BNO055_Registers::SELFTEST_RESULT_ADDR, &tmp);
		if (status == HAL_StatusTypeDef::HAL_OK) {
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

}
// If the function reaches this point, it means there was an error
// setting the page or Operation mode or reading the self-test result
last_error = SensorErrors::SELF_TEST_FAIL;
return false;
}

bool resetDevice();

bool BNO055_HAL::setPage(BNO055_PAGE_ID page) {
uint8_t status;

// check if the requested page is different. if it is then change
if (_deviceInfo.pageID != page) {
// dont know why the "page" enum with an underlying type "uint8_t" doesnt work.. so for now ill static cast it
	status = write8(BNO055_Registers::PAGE_ID_ADDR, page);
	if (status == HAL_StatusTypeDef::HAL_OK) {
		_deviceInfo.pageID = page;
		return true;
	}
}
return false;

}

bool BNO055_HAL::setOperationMode(BNO055_OPR_MODE mode) {

uint8_t status;
if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
	setPage(BNO055_PAGE_ID::PAGE_ZERO);
}
if (_deviceInfo.oprMode != mode
		&& _deviceInfo.pageID == BNO055_PAGE_ID::PAGE_ZERO) {

	status = write8(BNO055_Registers::OPR_MODE_ADDR, mode);
	if (status == HAL_StatusTypeDef::HAL_OK
			&& mode == BNO055_OPR_MODE::OPERATION_MODE_CONFIG) {

		_deviceInfo.oprMode = mode; // save mode

		// p22 table 3-6: Operation moder switching time
		HAL_Delay(19); // delay needed if its CONFIG mode

		return true;
	} else {
		_deviceInfo.oprMode = mode; // save mode
		HAL_Delay(7);
	}
}

return false;
}

bool BNO055_HAL::setExternalCrystalUse(bool state) {
uint8_t status;
if (_deviceInfo.pageID != BNO055_PAGE_ID::PAGE_ZERO) {
	setPage(BNO055_PAGE_ID::PAGE_ZERO);
}
uint8_t tmp = 0;
status = read8(BNO055_Registers::SYS_TRIGGER_ADDR, &tmp);
if (status == HAL_StatusTypeDef::HAL_OK) {
	if (state == true) {
		tmp |= 0x80;  // Set the MSB (most significant bit) of tmp to 1
	} else {
		tmp &= 0x0;  // Clear the MSB of tmp
	}
}

status = write8(BNO055_Registers::SYS_TRIGGER_ADDR, tmp);
if (status == HAL_StatusTypeDef::HAL_OK) {
	HAL_Delay(700);
	return true;
}
return false;

}

bool BNO055_HAL::unitSelect(BNO055_ACCEL_UNITS accelUnits,
	BNO055_GYRO_UNITS gyroUnits, BNO055_TEMP_UNITS tempUnits) {

uint8_t status = 0;

uint8_t unitSel = (0x00 << 7) | // Orientation Windows
		(tempUnits << 4) | // Temperature = ℃ (Celsius)
		(gyroUnits << 1) | // Gyro = Radians
		(accelUnits << 0); // Accelerometer = m/s^2

status = write8(BNO055_Registers::UNIT_SEL_ADDR, unitSel);

if (status == HAL_StatusTypeDef::HAL_OK) {
	return true;
} else {
	return false;
}
}

bool BNO055_HAL::setAccelConfig(BNO055_ACCEL_PWR_MODE accelPowerMode,
	BNO055_ACCEL_BW accelBWMode, BNO055_ACCEL_RANGE accelRange) {

uint8_t status = 0;

// Set the BNO055 sensor to configuration mode before making any changes
setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG);

// Set the register page to PAGE_ONE, where the accelerometer configuration registers are located
setPage(BNO055_PAGE_ID::PAGE_ONE);

// Local variable to store the combined configuration settings before writing to the register
uint8_t tmp = 0;

// Use the BNO055_SET_BITSLICE macro to set specific bits in 'tmp' based on the provided configuration parameters

// Set the accelerometer power mode bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_ACCEL_POWER_MODE, accelPowerMode);

// Set the accelerometer bandwidth bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_ACCEL_BW, accelBWMode);

// Set the accelerometer range bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_ACCEL_RANGE, accelRange);

// Write the combined configuration settings (stored in 'tmp') to the accelerometer configuration register
status = write8(BNO055_Registers::ACCEL_CONFIG_ADDR, tmp);

// Check if the write operation was successful (HAL_OK)
if (status == HAL_StatusTypeDef::HAL_OK) {

	// If the write was successful, set 'last_error' to NO_ERRORS and return true
	last_error = SensorErrors::NO_ERRORS;
	return true;
} else {
	// If there was an error in the write operation, set 'last_error' to BUS_FAULT and return false
	last_error = SensorErrors::BUS_FAULT;
	return false;
}

}

bool BNO055_HAL::setMagConfig(BNO055_MAG_PWR_MODE magPowerMode,
	BNO055_MAG_OPR_MODE magOPRMode, BNO055_MAG_OUTPUT_RATE magOutputRate) {

uint8_t status = 0;

// Set the BNO055 sensor to configuration mode before making any changes
setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG);

// Set the register page to PAGE_ONE, where the magnetometer configuration registers are located
setPage(BNO055_PAGE_ID::PAGE_ONE);

// Local variable to store the combined configuration settings before writing to the register
uint8_t tmp = 0;

// Set the magnetometer power mode bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_MAG_POWER_MODE, magPowerMode);

// Set the magnetometer operation mode bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_MAG_OPERATION_MODE, magOPRMode);

// Set the magnetometer range bits in 'tmp'
tmp = BNO055_SET_BITSLICE(tmp, BNO055_MAG_DATA_OUTPUT_RATE, magOutputRate);

// Write the combined configuration settings (stored in 'tmp') to the magnetometer configuration register
status = write8(BNO055_Registers::ACCEL_CONFIG_ADDR, tmp);

// Check if the write operation was successful (HAL_OK)
if (status == HAL_StatusTypeDef::HAL_OK) {

	// If the write was successful, set 'last_error' to NO_ERRORS and return true
	last_error = SensorErrors::NO_ERRORS;
	return true;
} else {
	// If there was an error in the write operation, set 'last_error' to BUS_FAULT and return false
	last_error = SensorErrors::BUS_FAULT;
	return false;
}

}

// Function to set gyroscope configuration in the BNO055 sensor
bool BNO055_HAL::setGyroConfig(BNO055_GYRO_PWR_MODE gyroPowerMode,
	BNO055_GYRO_BW gyroBWMode, BNO055_GYRO_RANGE gyroRange) {

uint8_t status = 0;

// Set the BNO055 sensor to configuration mode before making any changes
setOperationMode(BNO055_OPR_MODE::OPERATION_MODE_CONFIG);

// Set the register page to PAGE_ONE, where the gyroscope configuration registers are located
setPage(BNO055_PAGE_ID::PAGE_ONE);

// Local variable to store the combined configuration settings before writing to the register
uint8_t tmp[2] = { 0 }; // Initialize to 0

// Set the gyroscope bandwidth bits in 'tmp'
tmp[0] = BNO055_SET_BITSLICE(tmp[0], BNO055_GYRO_BW, gyroBWMode);

// Set the gyroscope range bits in 'tmp'
tmp[0] = BNO055_SET_BITSLICE(tmp[0], BNO055_GYRO_RANGE, gyroRange);

// Set the gyroscope power mode bits in 'tmp'
tmp[1] = BNO055_SET_BITSLICE(tmp[1], BNO055_GYRO_POWER_MODE, gyroPowerMode);

// Write the combined configuration settings (stored in 'tmp') to the gyroscope configuration register
status = writeLen(BNO055_Registers::GYRO_CONFIG_ADDR, *tmp, 2);

// Check if the write operation was successful (HAL_OK)
if (status == HAL_StatusTypeDef::HAL_OK) {
	// If the write was successful, set 'last_error' to NO_ERRORS and return true
	last_error = SensorErrors::NO_ERRORS;
	return true;
} else {
	// If there was an error in the write operation, set 'last_error' to BUS_FAULT and return false
	last_error = SensorErrors::BUS_FAULT;
	return false;
}
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

HAL_StatusTypeDef BNO055_HAL::writeLen(uint8_t regAddr, uint8_t data,
	uint8_t length) {

// Start the I2C memory write operation using interrupts
HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(&_i2cHandle,
BNO055_I2C_ADDR, regAddr,
I2C_MEMADD_SIZE_8BIT, &data, length);

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
last_error = SensorErrors::NO_ERRORS;

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

last_error = SensorErrors::NO_ERRORS;
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

last_error = SensorErrors::NO_ERRORS;
// Return HAL_OK to indicate successful data read.
return HAL_OK;
}
