/* AutoDrift — MPU-6050 driver (ATmega328P, I2C)
 * Reads GYRO_ZOUT via burst read; calibration removes static bias. */

#include "config.h"
#include "mpu6050.h"
#include "i2c.h"

#include <util/delay.h>
#include <stdint.h>

static int16_t gyro_z_offset = 0;

static int16_t read_raw_z(void)
{
    uint8_t buf[2];
    i2c_read_bytes(MPU6050_ADDR, MPU_GYRO_ZOUT_H, buf, 2);
    return (int16_t)((uint16_t)(buf[0] << 8) | buf[1]);
}

void mpu6050_init(void)
{
    i2c_write_register(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00); /* wake up */
    i2c_write_register(MPU6050_ADDR, MPU_GYRO_CONFIG, 0x00); /* ±250 deg/s */
    i2c_write_register(MPU6050_ADDR, MPU_DLPF_CONFIG, 0x03); /* DLPF ~42 Hz */
    _delay_ms(100);
}

void mpu6050_calibrate(void)
{
    int32_t sum = 0;
    for (uint8_t i = 0; i < 200U; i++) {
        sum += (int32_t)read_raw_z();
        _delay_ms(2); /* 200 × 2ms = 400ms total */
    }
    gyro_z_offset = (int16_t)(sum / 200L);
}

int16_t mpu6050_read_gyro_z_raw(void)
{
    return read_raw_z() - gyro_z_offset;
}

float mpu6050_get_yaw_rate(void)
{
    return (float)mpu6050_read_gyro_z_raw() / 131.0f; /* 131 LSB/(deg/s) at ±250 range */
}
