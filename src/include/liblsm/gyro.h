#ifndef __GYRO_H__
#define __GYRO_H__

#define GYRO_SLAVE_ADDRESS 0x6B
#define GYRO_ID_ADDRESS 0x0F /*WhoAmI (ID) register*/
#define GYRO_ID_RETURN 0x69 /*Value of WhoAmI register*/

/*Need to i2c write this in gyro_init*/
#define GYRO_DLPF_FS_ADDRESS 0x16 /*"full-scale" config register*/
#define GYRO_DLPF_FS_VALUE 0x18 /*magic value for "full-scale" meas., req'd to run*/

#define GYRO_PWRMGT_ADDRESS 0x3E /* power management register */
#define GYRO_PWRMGT_VALUE   0x40 /* SLEEP */

/*Data registers*/
#define GYRO_TEMPH_ADDRESS 0x1B
#define GYRO_TEMPL_ADDRESS 0x1C
#define GYRO_XH_ADDRESS 0x1D
#define GYRO_XL_ADDRESS 0x1E
#define GYRO_YH_ADDRESS 0x1F
#define GYRO_YL_ADDRESS 0x20
#define GYRO_ZH_ADDRESS 0x21
#define GYRO_ZL_ADDRESS 0x22

#define TEMPH 0
#define TEMPL 1
#define XH 2
#define XL 3
#define YH 4
#define YL 5
#define ZH 6
#define ZL 7

typedef struct {
  int x;
  int y;
  int z;
} gyro_t;

void gyro_init_pedom_int(void);
void gyro_init_tap_int(void);
void gyro_init_tilt_int(void);
void gyro_init_pedom_poll(void);
void gyro_init_raw(void);
void gyro_init_tap_drdy(void);
void gyro_init_fifo_tap(void);


uint16_t read_pedometer_steps(void);
void read_raw_gyro(uint16_t *, uint16_t *, uint16_t *);
void read_raw_accel(uint16_t *, uint16_t *, uint16_t *);
uint8_t read_drdy_status(void);
uint8_t read_fifo_lvl(void);
uint16_t read_fifo_val(void);
uint8_t read_fifo_thr(void);
void read_fifo_trio(uint16_t *x, uint16_t *y, uint16_t *z);
void fifo_clear(void);
void lsm_reset(void);
void lsm_reboot(void);
uint8_t read_tap_src(void);
void fifo_reset(void);
void dump_fifo(uint8_t *output, uint16_t dump_level);

#endif /*GYRO_H__*/
