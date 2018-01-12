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

  PRINTF("Wrote %x, read %x \r\n", dataToWrite, dataRead);

  // May need to add in write to ODR bits here... maybe not though

  set_slave_address(GYRO_SLAVE_ADDRESS);
  write_reg(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
  
  set_slave_address(GYRO_SLAVE_ADDRESS); 
  write_reg(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);

  set_slave_address(GYRO_SLAVE_ADDRESS); 
  write_reg(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10);

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

  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
  
  set_slave_address(GYRO_SLAVE_ADDRESS);  
  write_reg(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
  
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


uint16_t read_pedometer_steps(void) {
  set_slave_address(GYRO_SLAVE_ADDRESS);
  uint8_t temp = read_reg(LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  uint16_t stepsTaken = ((uint16_t) temp) << 8;
  PRINTF("first set steps take = %x \r\n",stepsTaken);
  set_slave_address(GYRO_SLAVE_ADDRESS);
  temp = read_reg(LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepsTaken |= temp;
  PRINTF("second set steps = %x \r\n",stepsTaken);
  return stepsTaken;
}

