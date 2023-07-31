/*
 * bno055_hal.h
 *
 *  Created on: Jul 4, 2023
 *      Author: ndanilo8
 */

#ifndef BNO055_HAL_H_
#define BNO055_HAL_H_

#include "BNO055Data.h"
#include "Sensors/Sensor.h"
#include <stm32f4xx_hal.h>

class BNO055_HAL: public Sensor<BNO055Data> {

public:

	BNO055_HAL(I2C_HandleTypeDef &i2cHandle);

	bool init() override;

	bool selfTest() override;

#define BNO055_I2C_ADDR (0x28 << 1)
#define  BNO055_GEN_READ_WRITE_LENGTH              ((uint8_t)1)
// #define  BNO055_SHIFT_EIGHT_BITS                   ((uint8_t)8)

	enum BNO055_Registers : uint8_t {
		/***************************************************/
		/**\name    REGISTER ADDRESS DEFINITION  */
		/***************************************************/
		/* Page id register definition*/
		PAGE_ID_ADDR = 0X07,

		/* PAGE0 REGISTER DEFINITION START*/
		CHIP_ID_ADDR = 0x00,
		ACCEL_REV_ID_ADDR = 0x01,
		MAG_REV_ID_ADDR = 0x02,
		GYRO_REV_ID_ADDR = 0x03,
		SW_REV_ID_LSB_ADDR = 0x04,
		SW_REV_ID_MSB_ADDR = 0x05,
		BL_REV_ID_ADDR = 0X06,

		/* Accel data register*/
		ACCEL_DATA_X_LSB_ADDR = 0X08,
		ACCEL_DATA_X_MSB_ADDR = 0X09,
		ACCEL_DATA_Y_LSB_ADDR = 0X0A,
		ACCEL_DATA_Y_MSB_ADDR = 0X0B,
		ACCEL_DATA_Z_LSB_ADDR = 0X0C,
		ACCEL_DATA_Z_MSB_ADDR = 0X0D,

		/*Mag data register*/
		MAG_DATA_X_LSB_ADDR = 0X0E,
		MAG_DATA_X_MSB_ADDR = 0X0F,
		MAG_DATA_Y_LSB_ADDR = 0X10,
		MAG_DATA_Y_MSB_ADDR = 0X11,
		MAG_DATA_Z_LSB_ADDR = 0X12,
		MAG_DATA_Z_MSB_ADDR = 0X13,

		/*Gyro data registers*/
		GYRO_DATA_X_LSB_ADDR = 0X14,
		GYRO_DATA_X_MSB_ADDR = 0X15,
		GYRO_DATA_Y_LSB_ADDR = 0X16,
		GYRO_DATA_Y_MSB_ADDR = 0X17,
		GYRO_DATA_Z_LSB_ADDR = 0X18,
		GYRO_DATA_Z_MSB_ADDR = 0X19,

		/* Temperature data register*/
		TEMP_ADDR = 0X34,

		/* Status registers*/
		CALIB_STAT_ADDR = 0X35,
		SELFTEST_RESULT_ADDR = 0X36,
		INTR_STAT_ADDR = 0X37,
		SYS_CLK_STAT_ADDR = 0X38,
		SYS_STAT_ADDR = 0X39,
		SYS_ERR_ADDR = 0X3A,

		/* Unit selection register*/
		UNIT_SEL_ADDR = 0X3B,
		DATA_SELECT_ADDR = 0X3C,

		/* Mode registers*/
		OPR_MODE_ADDR = 0X3D,
		PWR_MODE_ADDR = 0X3E,

		SYS_TRIGGER_ADDR = 0X3F,
		TEMP_SOURCE_ADDR = 0X40,

		/* Axis remap registers*/
		AXIS_MAP_CONFIG_ADDR = 0X41,
		AXIS_MAP_SIGN_ADDR = 0X42,

		/* PAGE0 REGISTERS DEFINITION END*/
		/* PAGE1 REGISTERS DEFINITION START*/
		/* Configuration registers*/
		ACCEL_CONFIG_ADDR = 0X08,
		MAG_CONFIG_ADDR = 0X09,
		GYRO_CONFIG_ADDR = 0X0A,
		GYRO_MODE_CONFIG_ADDR = 0X0B,
		ACCEL_SLEEP_CONFIG_ADDR = 0X0C,
		GYRO_SLEEP_CONFIG_ADDR = 0X0D,
		MAG_SLEEP_CONFIG_ADDR = 0x0E,

		/* Interrupt registers*/
		INT_MASK_ADDR = 0X0F,
		INT_ADDR = 0X10,
		ACCEL_INTR_SETTINGS_ADDR = 0X12,
		GYRO_INTR_SETING_ADDR = 0X17,

	/* PAGE1 REGISTERS DEFINITION END*/
	};

	/*ST_ACCEL register*/
#define BNO055_SELFTEST_ACCEL_POS                 (0)
#define BNO055_SELFTEST_ACCEL_MSK                 (0X01)
#define BNO055_SELFTEST_ACCEL_LEN                 (1)
	//	#define BNO055_SELFTEST_ACCEL_REG                 BNO055_SELFTEST_RESULT_ADDR
	/*ST_MAG register*/
#define BNO055_SELFTEST_MAG_POS                   (1)
#define BNO055_SELFTEST_MAG_MSK                   (0X02)
#define BNO055_SELFTEST_MAG_LEN                   (1)
	//	#define BNO055_SELFTEST_MAG_REG                   BNO055_SELFTEST_RESULT_ADDR

	/*ST_GYRO register*/
#define BNO055_SELFTEST_GYRO_POS                  (2)
#define BNO055_SELFTEST_GYRO_MSK                  (0X04)
#define BNO055_SELFTEST_GYRO_LEN                  (1)
	//	#define BNO055_SELFTEST_GYRO_REG                  BNO055_SELFTEST_RESULT_ADDR

	/*ST_MCU register*/
#define BNO055_SELFTEST_MCU_POS                   (3)
#define BNO055_SELFTEST_MCU_MSK                   (0X08)
#define BNO055_SELFTEST_MCU_LEN                   (1)
	//	#define BNO055_SELFTEST_MCU_REG                   BNO055_SELFTEST_RESULT_ADDR

	/*Self Test register*/
#define BNO055_SELFTEST_POS                       (0)
#define BNO055_SELFTEST_MSK                       (0X01)
#define BNO055_SELFTEST_LEN                       (1)
//#define BNO055_SELFTEST_REG                       BNO055_SYS_TRIGGER_ADDR

	/* System Status registers*/
#define BNO055_SYS_STAT_CODE_POS                  (0)
#define BNO055_SYS_STAT_CODE_MSK                  (0XFF)
#define BNO055_SYS_STAT_CODE_LEN                  (8)
//#define BNO055_SYS_STAT_CODE_REG                  BNO055_SYS_STAT_ADDR

	/* Temperature register*/
#define BNO055_TEMP_POS                           (0)
#define BNO055_TEMP_MSK                           (0xFF)
#define BNO055_TEMP_LEN                           (8)
	//#define BNO055_TEMP_REG                           BNO055_TEMP_ADDR

	/* Configuration registers*/
	/* Accel range configuration register*/
#define BNO055_ACCEL_RANGE_POS                 (0)
#define BNO055_ACCEL_RANGE_MSK                 (0X03)
#define BNO055_ACCEL_RANGE_LEN                 (2)
//	#define BNO055_ACCEL_RANGE_REG                 BNO055_ACCEL_CONFIG_ADDR

	/* Accel bandwidth configuration register*/
#define BNO055_ACCEL_BW_POS                    (2)
#define BNO055_ACCEL_BW_MSK                    (0X1C)
#define BNO055_ACCEL_BW_LEN                    (3)
//	#define BNO055_ACCEL_BW_REG                    BNO055_ACCEL_CONFIG_ADDR

	/* Accel power mode configuration register*/
#define BNO055_ACCEL_POWER_MODE_POS            (5)
#define BNO055_ACCEL_POWER_MODE_MSK            (0XE0)
#define BNO055_ACCEL_POWER_MODE_LEN            (3)
//	#define BNO055_ACCEL_POWER_MODE_REG            BNO055_ACCEL_CONFIG_ADDR

	/* Mag data output rate configuration register*/
#define BNO055_MAG_DATA_OUTPUT_RATE_POS        (0)
#define BNO055_MAG_DATA_OUTPUT_RATE_MSK        (0X07)
#define BNO055_MAG_DATA_OUTPUT_RATE_LEN        (3)
//	#define BNO055_MAG_DATA_OUTPUT_RATE_REG        BNO055_MAG_CONFIG_ADDR

	/* Mag operation mode configuration register*/
#define BNO055_MAG_OPERATION_MODE_POS          (3)
#define BNO055_MAG_OPERATION_MODE_MSK          (0X18)
#define BNO055_MAG_OPERATION_MODE_LEN          (2)
//	#define BNO055_MAG_OPERATION_MODE_REG          BNO055_MAG_CONFIG_ADDR

	/* Mag power mode configuration register*/
#define BNO055_MAG_POWER_MODE_POS              (5)
#define BNO055_MAG_POWER_MODE_MSK              (0X60)
#define BNO055_MAG_POWER_MODE_LEN              (2)
//	#define BNO055_MAG_POWER_MODE_REG              BNO055_MAG_CONFIG_ADDR

	/* Gyro range configuration register*/
#define BNO055_GYRO_RANGE_POS                  (0)
#define BNO055_GYRO_RANGE_MSK                  (0X07)
#define BNO055_GYRO_RANGE_LEN                  (3)
//	#define BNO055_GYRO_RANGE_REG                  BNO055_GYRO_CONFIG_ADDR

	/* Gyro bandwidth configuration register*/
#define BNO055_GYRO_BW_POS                     (3)
#define BNO055_GYRO_BW_MSK                     (0X38)
#define BNO055_GYRO_BW_LEN                     (3)
//	#define BNO055_GYRO_BW_REG                     BNO055_GYRO_CONFIG_ADDR

	/* Gyro power mode configuration register*/
#define BNO055_GYRO_POWER_MODE_POS             (0)
#define BNO055_GYRO_POWER_MODE_MSK             (0X07)
#define BNO055_GYRO_POWER_MODE_LEN             (3)
//	#define BNO055_GYRO_POWER_MODE_REG             BNO055_GYRO_MODE_CONFIG_ADDR

	/* Page ID */
	enum BNO055_PAGE_ID : uint8_t {

		PAGE_ZERO = 0X00, PAGE_ONE = 0X01
	};

	/* Operation mode settings*/
	enum BNO055_OPR_MODE : uint8_t {

		OPERATION_MODE_CONFIG = 0X00, OPERATION_MODE_AMG = 0X07
	};

	/* Power mode*/
	enum BNO055_PWR_MODE : uint8_t {

		POWER_MODE_NORMAL = 0X00,
		POWER_MODE_LOWPOWER = 0X01,
		POWER_MODE_SUSPEND = 0X02

	};

	/* PAGE-1 definitions*/
	enum BNO055_ACCEL_RANGE : uint8_t {
		ACCEL_RANGE_2G = 0X00,
		ACCEL_RANGE_4G = 0X01,
		ACCEL_RANGE_8G = 0X02,
		ACCEL_RANGE_16G = 0X03
	};\

	/* Accel Bandwidth*/
	enum BNO055_ACCEL_BW : uint8_t {

		ACCEL_BW_7_81HZ = 0x00,
		ACCEL_BW_15_63HZ = 0x01,
		ACCEL_BW_31_25HZ = 0x02,
		ACCEL_BW_62_5HZ = 0X03,
		ACCEL_BW_125HZ = 0X04,
		ACCEL_BW_250HZ = 0X05,
		ACCEL_BW_500HZ = 0X06,
		ACCEL_BW_1000HZ = 0X07

	};

	/* Accel Power mode*/
	enum BNO055_ACCEL_PWR_MODE : uint8_t {

		ACCEL_NORMAL = 0X00,
		ACCEL_SUSPEND = 0X01,
		ACCEL_LOWPOWER_1 = 0X02,
		ACCEL_STANDBY = 0X03,
		ACCEL_LOWPOWER_2 = 0X04,
		ACCEL_DEEPSUSPEND = 0X05

	};

	/* Mag data output rate*/
	enum BNO055_MAG_OUTPUT_RATE : uint8_t {

		MAG_DATA_OUTRATE_2HZ = 0X00,
		MAG_DATA_OUTRATE_6HZ = 0X01,
		MAG_DATA_OUTRATE_8HZ = 0X02,
		MAG_DATA_OUTRATE_10HZ = 0X03,
		MAG_DATA_OUTRATE_15HZ = 0X04,
		MAG_DATA_OUTRATE_20HZ = 0X05,
		MAG_DATA_OUTRATE_25HZ = 0X06,
		MAG_DATA_OUTRATE_30HZ = 0X07

	};

	/* Mag Operation mode*/
	enum BNO055_MAG_OPR_MODE : uint8_t {

		MAG_OPERATION_MODE_LOWPOWER = 0X00,
		MAG_OPERATION_MODE_REGULAR = 0X01,
		MAG_OPERATION_MODE_ENHANCED_REGULAR = 0X02,
		MAG_OPERATION_MODE_HIGH_ACCURACY = 0X03

	};

	/* Mag power mode*/
	enum BNO055_MAG_PWR_MODE : uint8_t {

		MAG_POWER_MODE_NORMAL = 0X00,
		MAG_POWER_MODE_SLEEP = 0X01,
		MAG_POWER_MODE_SUSPEND = 0X02,
		MAG_POWER_MODE_FORCE_MODE = 0X03

	};

	/* Gyro range*/
	enum BNO055_GYRO_RANGE : uint8_t {
		GYRO_RANGE_2000DPS = 0x00,
		GYRO_RANGE_1000DPS = 0x01,
		GYRO_RANGE_500DPS = 0x02,
		GYRO_RANGE_250DPS = 0x03,
		GYRO_RANGE_125DPS = 0x04

	};

	/* Gyro Bandwidth*/
	enum BNO055_GYRO_BW : uint8_t {

		GYRO_BW_523HZ = 0x00,
		GYRO_BW_230HZ = 0x01,
		GYRO_BW_116HZ = 0x02,
		GYRO_BW_47HZ = 0x03,
		GYRO_BW_23HZ = 0x04,
		GYRO_BW_12HZ = 0x05,
		GYRO_BW_64HZ = 0x06,
		GYRO_BW_32HZ = 0x07

	};

	/* Gyro power mode*/
	enum BNO055_GYRO_PWR_MODE : uint8_t {

		GYRO_POWER_MODE_NORMAL = 0X00,
		GYRO_POWER_MODE_FASTPOWERUP = 0X01,
		GYRO_POWER_MODE_DEEPSUSPEND = 0X02,
		GYRO_POWER_MODE_SUSPEND = 0X03,
		GYRO_POWER_MODE_ADVANCE_POWERSAVE = 0X04

	};
	/* Axis remap values*/
	enum BNO055_AXIS_REMAP : uint8_t {

		REMAP_X_Y = 0X21,
		REMAP_Y_Z = 0X18,
		REMAP_Z_X = 0X06,
		REMAP_X_Y_Z_TYPE0 = 0X12,
		REMAP_X_Y_Z_TYPE1 = 0X09,
		DEFAULT_AXIS = 0X24

	};
	/* Axis remap sign */
	enum BNO055_AXIS_REMAP_SIGN : uint8_t {

		REMAP_AXIS_POSITIVE = 0X00, REMAP_AXIS_NEGATIVE = 0X01

	};
	/*
	 union BNO055RawData {
	 struct __attribute__((packed)) BNO055RawDataBits {
	 int16_t accelX;
	 int16_t accelY;
	 int16_t accelZ;
	 int16_t temp;
	 int16_t gyroX;
	 int16_t gyroY;
	 int16_t gyroZ;
	 int16_t magX;
	 int16_t magY;
	 int16_t magZ;
	 } bits;
	 int8_t bytes[20];
	 };*/

private:
	// I2C handle as a class member variable
	I2C_HandleTypeDef &_i2cHandle;
	// Whether the sensor has been initialized
	bool initialized = false;

	struct BNO055Config_t {

			uint8_t chipID;
			BNO055_PAGE_ID pageID;
			BNO055_OPR_MODE oprMode;
			uint8_t accelRevID;
			uint8_t magRevID;
			uint8_t gyroRevID;
			uint8_t blRevID;
			uint8_t swRevID;

		} _deviceInfo;

		struct BNO055SelfTest {

			uint8_t mcuState;
			uint8_t gyroState;
			uint8_t magState;
			uint8_t accelState;

		} _selfTestResult;

		struct BNO055SensorScales {
			uint16_t accel = 100.0; // meters per second
			uint16_t temp = 1.0; // Celsius
			//uint16_t gyro = 16.0; // DPS Degrees per sec
			uint16_t gyro = 900.0; // RPS Radians per sec
			uint16_t mag = 16.0;

		} _sensorScales;

	/* GET AND SET BITSLICE FUNCTIONS    */
#define BNO055_GET_BITSLICE(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)

#define BNO055_SET_BITSLICE(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | ((val << bitname##_POS) & bitname##_MSK))

	BNO055Data sampleImpl() override;

	void resetDevice();

	bool setPage(BNO055_PAGE_ID page);

	bool setOperationMode(BNO055_OPR_MODE mode);

	bool setExternalCrystralUse(bool state);

	bool setAccelPWRMode(BNO055_ACCEL_PWR_MODE powerMode);


	/*
	 * LOW LEVEL FUNCTIONS
	 * */

	/*
	 // Function to get a bitslice from a register value
	 uint8_t get_bitslice(uint8_t regvar, uint8_t mask, uint8_t pos) {
	 return (regvar & mask) >> pos;
	 }

	 // Function to set a bitslice in a register value
	 uint8_t set_bitslice(uint8_t regvar, uint8_t mask, uint8_t pos,
	 uint8_t val) {
	 return (regvar & ~mask) | ((val << pos) & mask);
	 }
	 */

	/*!
	 *  @brief
	 *  This API gives data to the given register and
	 *  the data is written in the corresponding register address
	 *
	 *  @param addr : Address of the register
	 *  @param data : Data to be written to the register
	 *
	 *
	 *  @return results of bus communication function
	 */

	HAL_StatusTypeDef write8(uint8_t regAddr, uint8_t data);

	HAL_StatusTypeDef read8(uint8_t regAddr, uint8_t *data);

	HAL_StatusTypeDef readLen(uint8_t regAddr, uint8_t *data, uint8_t length);

};

#endif /* BNO055_HAL_H_ */
