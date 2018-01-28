#include <msp430.h>
#include <stdio.h>

#include <libmspware/driverlib.h>

#include <libmsp/mem.h>
#include <libio/console.h>

#include "gyro.h"
#include "lsm6ds3.h"

const unsigned int gyroBytes = 8;
const unsigned int gyroIdBytes = 1;
volatile unsigned char rawGyroData[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
volatile unsigned char gyroId = 0;


void write_reg(uint8_t reg,uint8_t val) {
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while((UCB0CTLW0 & UCTXSTT)); // wait for addr transmission to finish

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = reg;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0TXBUF = val;

  while(!(UCB0IFG & UCTXIFG)); // wait for txbuf to empty
  UCB0CTLW0 |= UCTXSTP; // stop

  while (UCB0STATW & UCBBUSY); // wait until bus is quiet

  return;
}

uint8_t read_reg(uint8_t reg) {
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!

  // Query gyro reg
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

  UCB0TXBUF = reg; // fill txbuf with reg address

  while(!(UCB0IFG & UCTXIFG)); // wait until txbuf is empty

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);

  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG)); // wait until txbuf is empty
  uint8_t val = UCB0RXBUF; // read out of rx buf

  while (UCB0STATW & UCBBUSY); // hang out until bus is quiet

  return val;
}


void set_slave_address(uint8_t addr) {
  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = addr; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable
  while (UCB0STATW & UCBBUSY); // is bus busy? then wait!
}
void gyro_init_pedom_int(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //PRINTF("Wrote data \r\n");
  set_slave_address(GYRO_SLAVE_ADDRESS);

  uint8_t dataRead = read_reg(LSM6DS3_ACC_GYRO_CTRL1_XL);

  // May need to add in write to ODR bits here... maybe not though

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  // Decrease the debounce threshold but leave time at default
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_PEDO_DEB, 0x61);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x80);

  return;
}

void gyro_init_pedom_poll(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //PRINTF("Wrote data \r\n");
  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t dataRead = read_reg(LSM6DS3_ACC_GYRO_CTRL1_XL);


  // May need to add in write to ODR bits here... maybe not though

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  // Changes to try to add in step detection interrupt
  // Decrease the debounce threshold but leave time at default
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_PEDO_DEB, 0x61);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  //write_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);
  write_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x80);

  return;
}

void gyro_init_fifo_tap(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  // Set up the acceleromter
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set up the gyro
  dataToWrite = 0;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_245dps;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t dataRead = read_reg(LSM6DS3_ACC_GYRO_CTRL1_XL);

  PRINTF("Wrote %x, read %x \r\n", dataToWrite, dataRead);

  // May need to add in write to ODR bits here... maybe not though
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0F);
  //write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x01);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

#if 0
  // Direct tap out to INT1
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);
#endif

#if 1
  // Direct tap out to INT2
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_MD2_CFG, 0x48);
#endif
  // Set block data update bit so we can read the fifo level and hope we're not
  // squashing any important settings in the process...
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL3_C, 0x44);



  // FIFO configuration
  uint8_t tempFIFO_CTRL1 = 0;
  uint8_t tempFIFO_CTRL2 = 0;
  uint8_t tempFIFO_CTRL3 = 0;
  uint8_t tempFIFO_CTRL4 = 0;
  uint8_t tempFIFO_CTRL5 = 0;
  // Set bits [7:0] of threshold level
  tempFIFO_CTRL1 = 0x30;
  // Set bits [12:0] of threshold level and pedom-fifo settings
  tempFIFO_CTRL2 = 0x0;
  // Set FIFO decimation settings for gyro and accel
  tempFIFO_CTRL3 = 0x1;
  // Set FIFO decimation settings for sensorhub and temp sensor
  tempFIFO_CTRL4 = 0x0;
  // Set FIFO ODR ([6:3]) and operating mode ([2:0])
  tempFIFO_CTRL5 = 0x21;
  // Write all the settings at once
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL1, tempFIFO_CTRL1);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL2, tempFIFO_CTRL2);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);
  
  set_slave_address(GYRO_SLAVE_ADDRESS);
  dataRead = read_reg(LSM6DS3_ACC_GYRO_FIFO_CTRL5);
 
  // Stop on Fth enabled
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL4_C, 0x1);

  PRINTF("to ctrl5: %x, read %x \r\n",tempFIFO_CTRL5, dataRead);
#if 0
  // Send FIFO threshold interrupt to INT2
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT2_CTRL, 0x8);
#endif

#if 1
  // Send FIFO threshold interrupt to INT1
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x38);
#endif

  return;

}

void gyro_init_tap_drdy(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  // Set up the acceleromter
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set up the gyro
  dataToWrite = 0;
  dataToWrite = LSM6DS3_ACC_GYRO_FS_G_245dps;
  dataToWrite = LSM6DS3_ACC_GYRO_ODR_G_26Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t dataRead = read_reg(LSM6DS3_ACC_GYRO_CTRL1_XL);

  PRINTF("Wrote %x, read %x \r\n", dataToWrite, dataRead);

  // Send drdy signal to INT1 pin
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT2_CTRL, 0x01);

  // Mask off drdy signal?
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL4_C, 0x8);

  // May need to add in write to ODR bits here... maybe not though
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x03);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);
  return;
}

void gyro_init_tap_int(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  // Set up the acceleromter
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set up the gyro
  dataToWrite = 0;
  dataToWrite = LSM6DS3_ACC_GYRO_FS_G_245dps;
  dataToWrite = LSM6DS3_ACC_GYRO_ODR_G_104Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  //PRINTF("Wrote data \r\n");
  set_slave_address(GYRO_SLAVE_ADDRESS);

  uint8_t dataRead = read_reg(LSM6DS3_ACC_GYRO_CTRL1_XL);

  PRINTF("Wrote %x, read %x \r\n", dataToWrite, dataRead);

  // May need to add in write to ODR bits here... maybe not though
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x03);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);
  return;
}

void gyro_init_raw(void) {

  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  // Set up the accelerometer
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set up the gyro
  dataToWrite = 0;
  dataToWrite = LSM6DS3_ACC_GYRO_FS_G_245dps;
  dataToWrite = LSM6DS3_ACC_GYRO_ODR_G_104Hz;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

  return;
}

void gyro_init_tilt_int(void) {
  // Set slave address //
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = GYRO_SLAVE_ADDRESS; // Set slave address
  UCB0CTLW0 &= ~UCSWRST; // enable

  uint8_t temp = read_reg(GYRO_ID_ADDRESS);
  if(temp != GYRO_ID_RETURN) {
    PRINTF("Error initializing gyro!\r\n");
    while(1);
  }

  uint8_t dataToWrite = 0;

  // Set up the accelerometer
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);

  // Enable embedded functions
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3C);

  // Enable tilt detection
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x20);

  // Send tilt detection to INT1
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_MD1_CFG, 0x02);

  return;
}

uint16_t read_pedometer_steps(void) {
  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t temp = read_reg(LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  uint16_t stepsTaken = ((uint16_t) temp) << 8;
  //PRINTF("first set steps take = %x \r\n",stepsTaken);
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepsTaken |= temp;

  temp = 0;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_FUNC_SRC);
  if(temp & 0x10) {
    PRINTF("Step detected!\r\n");
  }
  //PRINTF("second set steps = %x \r\n",stepsTaken);
  return stepsTaken;
}


void read_raw_gyro(uint16_t *x, uint16_t *y, uint16_t *z) {
  uint8_t temp_l, temp_h;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTX_L_G);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTX_H_G);

  *x = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTY_L_G);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTY_H_G);

  *y = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTZ_L_G);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTZ_H_G);

  *z = (temp_h << 8) + temp_l;

  return;
}


void read_raw_accel(uint16_t *x, uint16_t *y, uint16_t *z) {
  uint8_t temp_l, temp_h;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTX_L_XL);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTX_H_XL);

  *x = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTY_L_XL);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTY_H_XL);

  *y = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_OUTZ_L_XL);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_OUTZ_H_XL);

  *z = (temp_h << 8) + temp_l;

  return;
}

uint8_t read_drdy_status(void) {
  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t temp;
  temp = read_reg(LSM6DS3_ACC_GYRO_STATUS_REG);
  temp = temp & 0x1;
  return temp;
}

// Note, this will return values from different sensors depending on how you
// have your FIFO configured. Check out the FIFO_PATTERN register to figure out
// exactly what you'll get.
void read_fifo_trio(uint16_t *x, uint16_t *y, uint16_t *z) {
  uint8_t temp_l, temp_h; 
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);

  *x = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);

  *y = (temp_h << 8) + temp_l;

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);

  *z = (temp_h << 8) + temp_l;

  return;
}

uint16_t read_fifo_val(void) {
  uint8_t temp_l, temp_h; 
  uint16_t x;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_l = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp_h = read_reg(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H);

  x = (temp_h << 8) + temp_l;

  return x;
}

// Returns the lower 8 bits of how many samples are left in the FIFO
// We're being lazy and not digging the upper bits out of STATUS2
uint8_t read_fifo_lvl(void) {
  uint8_t temp;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_FIFO_STATUS1);
  return temp;
}

uint8_t read_fifo_thr(void) {
  uint8_t temp;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_FIFO_STATUS2);
  
  return temp;
}

uint8_t read_tap_src(void) {
  uint8_t temp;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_TAP_SRC);
  return temp;
}

void fifo_clear(void) {
  uint16_t x;
  uint8_t temp = 0, temp2 = 0;
  while(!temp) {
    set_slave_address(GYRO_SLAVE_ADDRESS);
    temp = read_reg(LSM6DS3_ACC_GYRO_FIFO_STATUS2);
    // Check fifo empty flag
    //printf("temp = %u \r\n",temp);
    temp = temp & 0x10;
    x = read_fifo_val();
  }
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_FIFO_STATUS1);

  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp2 = read_reg(LSM6DS3_ACC_GYRO_FIFO_STATUS2);

  printf("status2 = %x status1 = %x\r\n",temp2,temp);
  return;
}

void lsm_reset(void) {
  //gyro in power down
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, 0x0);
  //accel in high performance
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL6_C, 0x0);
  //sw_reset
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL3_C, 0x1);
  //Wait until no longer in SW_RESET
  uint8_t temp = 1;
  set_slave_address(GYRO_SLAVE_ADDRESS);
  while(temp) {
    temp = read_reg(LSM6DS3_ACC_GYRO_CTRL3_C);
    temp = temp & 0x1;
    PRINTF("temp = %u \r\n",temp);
  }
  return;
}

void lsm_reboot(void) {
  // gyro in power down
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL2_G, 0x0);
  // accel in high performance
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL6_C, 0x0);
  // reboot bit
  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL3_C, 0x80);
  // Wait 20ms after this function!
  return;
}

