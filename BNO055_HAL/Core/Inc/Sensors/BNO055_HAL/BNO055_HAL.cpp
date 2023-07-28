/*
 * bno055_hal.cpp
 *
 *  Created on: Jul 24, 2023
 *      Author: ndanilo8
 */

#include "BNO055_HAL.h"

BNO055_HAL::BNO055_HAL(I2C_HandleTypeDef *i2cHandle) {
	// Save the I2C handle to the class member variable
	this->i2cHandle = i2cHandle;
}

bool BNO055_HAL::init() {
	//check if sensor already initialized
	if (initialized) {

		// log "Already Initialized"
		last_error = SensorErrors::ALREADY_INIT;
		return false;
	}

	// read chip id

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

/*
 * LOW LEVEL FUNCTIONS
 * */
HAL_StatusTypeDef BNO055_HAL::write8(uint8_t regAddr, uint8_t *data) {

	uint8_t status = HAL_I2C_Mem_Write_IT(BNO055_I2C_ADDR, regAddr,
	I2C_MEMADD_SIZE_8BIT, data, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		return status;
	}

	// Return HAL_OK to indicate successful data read.
	return HAL_OK;

}

HAL_StatusTypeDef BNO055_HAL::read8(uint8_t regAddr, uint8_t *data) {

	uint8_t status = HAL_I2C_Mem_Read_IT(BNO055_I2C_ADDR, regAddr,
			I2C_MEMADD_SIZE_8BIT, data, BNO055_GEN_READ_WRITE_LENGTH);

	// Check for I2C errors and handle them
	if (status != HAL_OK) {
		// Perform error handling, such as retries, error logging, etc.
		// For example, you might want to log the error or attempt a retry.

		// uint32_t error = HAL_I2C_GetError(i2cHandle);

		// For simplicity, you can return an error status code here if you want
		// to indicate the failure to the caller.
		return status;
	}

	// Return HAL_OK to indicate successful data read.
	return HAL_OK;
}

HAL_StatusTypeDef BNO055_HAL::readLen(uint8_t regAddr, uint8_t *data, uint8_t length){

	uint8_t status = HAL_I2C_Mem_Read_IT(BNO055_I2C_ADDR, regAddr,
				I2C_MEMADD_SIZE_8BIT, data, length);
	// Check for I2C errors and handle them
		if (status != HAL_OK) {
			// Perform error handling, such as retries, error logging, etc.
			// For example, you might want to log the error or attempt a retry.

			// uint32_t error = HAL_I2C_GetError(i2cHandle);

			// For simplicity, you can return an error status code here if you want
			// to indicate the failure to the caller.
			return status;
		}

		// Return HAL_OK to indicate successful data read.
		return HAL_OK;
}
