#ifndef __ACCL_H__
#define __ACCL_H__
#include "lsm6ds3.h"

void accelerometer_init();
void accelerometer_init_data_rate(LSM6DS3_ACC_GYRO_ODR_XL_t rate);
void accelerometer_read();
void accelerometer_disable();

#define ACCL_I2C_ADDRESS 0x6B
#define ACCL_ID_ADDRESS 0x0F /*WhoAmI (ID) register*/
#define ACCL_ID_RETURN 0x69 /*Value of WhoAmI register*/
#define XL_MASK 0x1

#endif
