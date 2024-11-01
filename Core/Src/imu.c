#include "imu.h"

#define I2C_ADDRESS                     0x68

#define REG_BANK_SEL                    0x7F

// USER BANK 1 REGISTERS
#define WHO_AM_I						0x00

#define PWR_MGMT_1						0x06

//#define ACC_PWR_CTRL                    0x7D
//#define ACC_PWR_CONF					0x7C
//
//#define INT1_INT2_MAP_DATA				0x58
//#define INT1_IO_CONF					0x53

#define TEMP_LSB_REGISTER               0x3A
#define TEMP_MSB_REGISTER               0x39

#define ACCEL_X_LSB_REGISTER              0x2E
#define ACCEL_X_MSB_REGISTER              0x2D
#define ACCEL_Y_LSB_REGISTER              0x30
#define ACCEL_Y_MSB_REGISTER              0x2F
#define ACCEL_Z_LSB_REGISTER              0x32
#define ACCEL_Z_MSB_REGISTER              0x31

#define GYRO_X_LSB_REGISTER             0x34
#define GYRO_X_MSB_REGISTER             0x33
#define GYRO_Y_LSB_REGISTER             0x36
#define GYRO_Y_MSB_REGISTER             0x35
#define GYRO_Z_LSB_REGISTER             0x38
#define GYRO_Z_MSB_REGISTER             0x37

//#define ACC_RANGE_REGISTER              0x41
//#define ACC_RANGE                       0x01
//// least significant 2 bits
//// 0x00 = +/- 3g
//// 0x01 = +/- 6g	**default
//// 0x02 = +/- 12g
//// 0x03 = +/- 24g
//
//#define GYRO_RANGE_REGISTER             0x0F
//#define GYRO_RANGE                      0x00
//// all 8 bits used, no need to read mask write
//// 0x00 = +/- 2000 deg/s	**default
//// 0x01 = +/- 1000 deg/s
//// 0x02 = +/- 500 deg/s
//// 0x03 = +/- 250 deg/s
//// 0x04 = +/- 125 deg/s
//
//// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
//// see: 4.4.1 and 5.3.10
//#define ACC_CONF						0x40
//#define ACC_BWP							0x0A	// "normal" mode			(default: 0x0A "normal")
//#define ACC_ODR							0x08	// 100Hz output data rate 	(default: 0x08 100Hz)
//
//// see: 5.5.6
//#define GYRO_BANDWIDTH_REGISTER			0x10
//#define GYRO_BANDWIDTH					0x00	// 2000Hz output data rate, 532Hz filter bandwidth
//												// (default: 0x00 2000Hz and 532 Hz)
//
//#define ACC_SELF_TEST					0x6D
//#define GYRO_SELF_TEST					0x3C
//
//#define ACC_SELF_TEST_READINGS			5		// get average of n readings
//#define ACC_SELF_TEST_DELAY_MS			5		// get a reading every n ms
//
//#define	GYRO_SOFTRESET					0x14
//#define	ACC_SOFTRESET					0x7E

#define TIMEOUT_MS              		1

extern I2C_HandleTypeDef hi2c3;

// convenience functions
HAL_StatusTypeDef I2C_Read_Byte(uint16_t DevAddress, uint16_t MemAddress, uint8_t *data) {
	uint8_t data_array[1];
	HAL_StatusTypeDef res = HAL_I2C_Mem_Read(&hi2c3, (DevAddress << 1) | 1, MemAddress, I2C_MEMADD_SIZE_8BIT, data_array, 1, TIMEOUT_MS);
    *data = data_array[0];
    return res;
}

HAL_StatusTypeDef I2C_Write_Byte(uint16_t DevAddress, uint16_t MemAddress, uint8_t data) {
	uint8_t data_array[1] = {data};
	return HAL_I2C_Mem_Write(&hi2c3, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT, data_array, 1, TIMEOUT_MS);
}

HAL_StatusTypeDef I2C_Read_Bytes(uint16_t DevAddress, uint16_t MemAddress, uint8_t *data, uint16_t Size) {
    return HAL_I2C_Mem_Read(&hi2c3, (DevAddress << 1) | 1, MemAddress, I2C_MEMADD_SIZE_8BIT, data, Size, TIMEOUT_MS);
}

HAL_StatusTypeDef I2C_Write_Bytes(uint16_t DevAddress, uint16_t MemAddress, uint8_t *data, uint16_t Size) {
	return HAL_I2C_Mem_Write(&hi2c3, DevAddress << 1, MemAddress, I2C_MEMADD_SIZE_8BIT, data, Size, TIMEOUT_MS);
}

void IMU_Init() {
    HAL_Delay(100);
	IMU_Change_User_Bank(0);
	// default startup is 0x41, we want to disable sleep
	// which is the 2nd byte so it becomes 0x01
    I2C_Write_Byte(I2C_ADDRESS, PWR_MGMT_1, 0x01);
    HAL_Delay(40);
}

void IMU_Change_User_Bank(uint8_t bank) {
    I2C_Write_Byte(I2C_ADDRESS, REG_BANK_SEL, bank);
}

uint8_t IMU_WhoAmI() {
	IMU_Change_User_Bank(0);
	// expect 0xE1
	uint8_t res;
	I2C_Read_Byte(I2C_ADDRESS, WHO_AM_I, &res);
	return res;
}

//void IMU_Settings() {
//    // setting accelerometer range
//    // as recommended by datasheet, we are reading this byte first to only modify the least significant 2 bits
//    uint8_t read;
//	I2C_Read_Byte(I2C_ADDRESS_ACCEL, ACC_RANGE_REGISTER, &read);
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_RANGE_REGISTER, (read & 0b11111100) | ACC_RANGE);
//
//    // setting gyro range
//    I2C_Write_Byte(I2C_ADDRESS_GYRO, GYRO_RANGE_REGISTER, GYRO_RANGE);
//
//    // setting accelerometer configuration
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_CONF, ACC_BWP << 4 | ACC_ODR);
//
//    // setting gyro configuration
//    // MSB here is always 1 and read-only. We're writing the 1 here just in case but it's probably unnecessary
//    I2C_Write_Byte(I2C_ADDRESS_GYRO, GYRO_BANDWIDTH_REGISTER, GYRO_BANDWIDTH | (1 << 7));
//}

double IMU_Read_Accel(uint8_t axis) {
	IMU_Change_User_Bank(0);

    // return unit: g's
    uint8_t data[2];

    if (axis == IMU_AXIS_X) {
    	I2C_Read_Bytes(I2C_ADDRESS, ACCEL_X_MSB_REGISTER, data, 2);
    } else if (axis == IMU_AXIS_Y) {
    	I2C_Read_Bytes(I2C_ADDRESS, ACCEL_Y_MSB_REGISTER, data, 2);
    } else if (axis == IMU_AXIS_Z) {
    	I2C_Read_Bytes(I2C_ADDRESS, ACCEL_Z_MSB_REGISTER, data, 2);
    }

    int16_t g = (data[0] << 8) | data[1];

    return g / 8192.0;
}

double IMU_Read_Gyro(uint8_t axis) {
	IMU_Change_User_Bank(0);

    // return unit: deg/s
    uint8_t data[2];

    if (axis == IMU_AXIS_X) {
    	I2C_Read_Bytes(I2C_ADDRESS, GYRO_X_MSB_REGISTER, data, 2);
    } else if (axis == IMU_AXIS_Y) {
    	I2C_Read_Bytes(I2C_ADDRESS, GYRO_Y_MSB_REGISTER, data, 2);
    } else if (axis == IMU_AXIS_Z) {
    	I2C_Read_Bytes(I2C_ADDRESS, GYRO_Z_MSB_REGISTER, data, 2);
    }

    int16_t rate = (data[0] << 8) | data[1];

    return rate / 65.5;
}

double IMU_Read_Temp() {
	IMU_Change_User_Bank(0);

    // return unit: celsius
    uint8_t data[2];
	I2C_Read_Bytes(I2C_ADDRESS, TEMP_MSB_REGISTER, data, 2);

    int16_t temp = (data[0] << 8) | data[1];

    return (temp - 21.0)/333.87 + 21.0;
}

//bool IMU_Accel_Self_Test() {
//	// see datasheet 4.6.1
//    // set max accel range
//	uint8_t read;
//	I2C_Read_Byte(I2C_ADDRESS_ACCEL, ACC_RANGE_REGISTER, &read);
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_RANGE_REGISTER, (read & 0b11111100) | 0x03);
//
//    // setting accelerometer configuration
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_CONF, 0xA7);
//
//    HAL_Delay(2);
//
//    // start positive self-test
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_SELF_TEST, 0x0D);
//    HAL_Delay(50);
//
//    // get average reading over a few ms
//    double avg_x_p = 0;
//    double avg_y_p = 0;
//    double avg_z_p = 0;
//
//    for (int i = 0; i < ACC_SELF_TEST_READINGS; i++) {
//    	uint8_t data[2];
//
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_X_LSB_REGISTER, data, 2);
//		avg_x_p = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0; // 24000.0 = 1000.0 * 16 * 1.5
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_Y_LSB_REGISTER, data, 2);
//		avg_y_p = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0;
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_Z_LSB_REGISTER, data, 2);
//		avg_z_p = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0;
//
//    	HAL_Delay(ACC_SELF_TEST_DELAY_MS);
//    }
//
//    // start negative self-test
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_SELF_TEST, 0x09);
//    HAL_Delay(50);
//
//    // get average reading over a few ms
//    double avg_x_n = 0;
//    double avg_y_n = 0;
//    double avg_z_n = 0;
//
//    for (int i = 0; i < ACC_SELF_TEST_READINGS; i++) {
//    	uint8_t data[2];
//
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_X_LSB_REGISTER, data, 2);
//		avg_x_n = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0; // 24000.0 = 1000.0 * 16 * 1.5
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_Y_LSB_REGISTER, data, 2);
//		avg_y_n = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0;
//		I2C_Read_Bytes(I2C_ADDRESS_ACCEL, ACC_Z_LSB_REGISTER, data, 2);
//		avg_z_n = (24000.0 * ((data[1] << 8) | data[0])) / 32768.0;
//
//    	HAL_Delay(ACC_SELF_TEST_DELAY_MS);
//    }
//
//    // stop self-test
//    I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_SELF_TEST, 0x00);
//
//    bool accel_test_pass = (avg_x_p - avg_x_n > 1000 * ACC_SELF_TEST_READINGS) &&
//    					   (avg_y_p - avg_y_n > 1000 * ACC_SELF_TEST_READINGS) &&
//						   (avg_z_p - avg_z_n > 500 * ACC_SELF_TEST_READINGS);
//
//    HAL_Delay(50);
//    return accel_test_pass;
//}
//
//bool IMU_Gyro_Self_Test() {
//	// see datasheet 4.6.2 and 5.5.14
//	uint8_t read;
//	I2C_Read_Byte(I2C_ADDRESS_GYRO, GYRO_SELF_TEST, &read);
//	I2C_Write_Byte(I2C_ADDRESS_GYRO, GYRO_SELF_TEST, read | 0x01); // write 0x01 to trig_bist; LSB of GYRO_SELF_TEST
//
//    bool bist_rdy = 0;
//    while (!bist_rdy) {
//    	I2C_Read_Byte(I2C_ADDRESS_GYRO, GYRO_SELF_TEST, &read);
//    	bist_rdy = read & 0b0010;
//    	HAL_Delay(1);
//    }
//
//	bool gyro_test_pass = !(read & 0b0100);
//	I2C_Write_Byte(I2C_ADDRESS_GYRO, GYRO_SELF_TEST, read & ~0b1); // write 0x00 to trig_bist, probably unnecessary but why not
//
//	return gyro_test_pass;
//}
//
//bool IMU_Gyro_OK() {
//	/*
//	 * Gyro has a constant 'self-test' bit, see 4.6.2 and 5.5.14
//	 *
//	 * NOTE:
//	 * Unsure whether this functionality is on if 0th bit of GYRO_SELF_TEST (trig_bist) is set.
//	 * */
//    uint8_t read;
//    I2C_Read_Byte(I2C_ADDRESS_GYRO, GYRO_SELF_TEST, &read);
//
//    // 4th bit is the rate_ok bit
//    return read & 0b00010000;
//}
//
//void IMU_Soft_Reset() {
//	I2C_Write_Byte(I2C_ADDRESS_GYRO, GYRO_SOFTRESET, 0xB6);
//	HAL_Delay(30);
//
//
//	I2C_Write_Byte(I2C_ADDRESS_ACCEL, ACC_SOFTRESET, 0xB6);
//	HAL_Delay(1);
//
//	/*
//	 * if accel reset doesnt work: PWR_CONF.adv_power_save=1 PWR_CTRL.acc_en=0 PWR_CTRL.aux_en=0 before softreset
//	*/
//}
