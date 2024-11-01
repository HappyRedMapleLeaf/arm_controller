#ifndef INC_IMU_H_
#define INC_IMU_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define IMU_AXIS_X 0
#define IMU_AXIS_Y 1
#define IMU_AXIS_Z 2

void IMU_Init();

void IMU_Change_User_Bank(uint8_t bank);

uint8_t IMU_WhoAmI();


//void IMU_Settings();
//void IMU_Soft_Reset();
double IMU_Read_Accel(uint8_t axis);
double IMU_Read_Gyro(uint8_t axis);
double IMU_Read_Temp();

///* Recommended to reset and change settings after accelerometer self-test.
// * This self-test delays at least 150ms. */
//bool IMU_Accel_Self_Test();
//
///* This self-test delays until completion, unsure what normal self-test times are. */
//bool IMU_Gyro_Self_Test();
//
///* Gyro has a constant 'self-test' bit, see 4.6.2 and 5.5.14 */
//bool IMU_Gyro_OK();

#endif /* INC_IMU_H_ */
