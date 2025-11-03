#include "imu.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

static void _imu_poll_accelerometer(void) {
    uint8_t raw_data[6];
    imu_read_burst((char*)raw_data, 6, IMU_BANK_ACCEL, IMU_REG_ACCEL_XOUT_H);

    m_state_prev = m_state_cur;
    m_state_cur.acc_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    m_state_cur.acc_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    m_state_cur.acc_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
}

void imu_setup(void) {
    HAL_Delay(100);
    imu_select_user_regbank(0);
    imu_write_single(IMU_REGx06_PWR_MGMT1, IMU_BIT7_PWR_MGMT1_DEVICE_RESET);
    HAL_Delay(100);
    imu_write_single(IMU_REGx06_PWR_MGMT1,
                     IMU_REGx06_DEFAULT_PWR_MGMT1 & ~IMU_BIT6_PWR_MGMT1_SLEEP);
    imu_write_single(IMU_REGx07_PWR_MGMT2, IMU_BITS2to0_PWR_MGMT2_DISABLE_GYRO);
}

void imu_write_single(uint8_t reg_addr, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c2, IMU_I2C_ADDR << 1, reg_addr,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

void imu_read_single(uint8_t reg_addr, uint8_t* value) {
    HAL_I2C_Mem_Read(&hi2c2, IMU_I2C_ADDR << 1, reg_addr,
                     I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

void imu_read_burst(char* buf, uint8_t len, uint8_t bank, uint8_t reg) {
    imu_select_user_regbank(bank);
    HAL_I2C_Mem_Read(&hi2c2, IMU_I2C_ADDR << 1, reg,
                     I2C_MEMADD_SIZE_8BIT, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

void imu_select_user_regbank(unsigned char bank_num) {
    imu_write_single(IMU_REG_BANK_SEL, bank_num << 4);
}

void imu_get_whoami(unsigned char* whoami) {
    imu_select_user_regbank(0);
    imu_read_single(IMU_REG_WHOAMI, whoami);
}

void imu_poll_state(void) {
    _imu_poll_accelerometer();
}
