#ifndef __ACCL_H__
#define __ACCL_H__

void accelerometer_init();
void accelerometer_read();
void accelerometer_disable();

#define ACCL_I2C_ADDRESS 0x6B
#define ACCL_ID_ADDRESS 0x0F /*WhoAmI (ID) register*/
#define ACCL_ID_RETURN 0x69 /*Value of WhoAmI register*/
#define XL_MASK 0x1

#endif
