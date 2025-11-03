#ifndef SECBOT_I2C_TO_IMU_H
#define SECBOT_I2C_TO_IMU_H

#include <stdint.h>
#include "state.h"

#if (STATE_TYPE == STATE_IS_IMU)

#define IMU_I2C_ADDR 0x69

#define IMU_REG_WHOAMI 0x00
#define IMU_VAL_WHOAMI 0xEA
#define IMU_REG_BANK_SEL 0x7F

#define IMU_REGx06_PWR_MGMT1  0x06
#define IMU_REGx07_PWR_MGMT2  0x07
#define IMU_REGx06_DEFAULT_PWR_MGMT1 0x41
#define IMU_BIT7_PWR_MGMT1_DEVICE_RESET 0x80
#define IMU_BIT6_PWR_MGMT1_SLEEP 0x40
#define IMU_BITS2to0_PWR_MGMT2_DISABLE_GYRO  0x07

#define IMU_BANK_ACCEL 0x00
#define IMU_REG_ACCEL_XOUT_H 0x2D
#define IMU_REG_ACCEL_YOUT_H 0x2F
#define IMU_REG_ACCEL_ZOUT_H 0x31

void imu_setup(void);
void imu_write_single(uint8_t reg_addr, uint8_t value);
void imu_read_single(uint8_t reg_addr, uint8_t* value);
void imu_read_burst(char* buf, uint8_t len, uint8_t bank, uint8_t reg);
void imu_select_user_regbank(unsigned char bank_num);
void imu_get_whoami(unsigned char* whoami);
void imu_poll_state(void);

#endif
#endif
