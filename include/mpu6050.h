#ifndef MPU6050_H
#define MPU6050_H

/* =========================================================
 *  AutoDrift — MPU-6050 Gyroscope Driver
 *  Target: ATmega324P @ 16 MHz
 *  Only the Z-axis (yaw rate) is used for drift correction.
 * ========================================================= */

#include <stdint.h>

/**
 * Initialise the MPU-6050:
 *   - Wake from sleep (PWR_MGMT_1 = 0x00)
 *   - Set gyro full-scale range ±250 deg/s (GYRO_CONFIG = 0x00)
 *   - Enable 42 Hz low-pass filter (DLPF_CONFIG = 0x03)
 *   - Wait 100 ms for sensor to settle
 */
void mpu6050_init(void);

/**
 * Calibrate the gyro Z offset.
 * Call once while the car is STATIONARY.
 * Averages 200 samples (≈ 400 ms) and stores the offset internally.
 */
void mpu6050_calibrate(void);

/**
 * Read the raw 16-bit signed GYRO_Z value minus the calibration offset.
 * Range: ±32767 LSB at ±250 deg/s → LSB/LSB sensitivity = 131 LSB/(deg/s).
 */
int16_t mpu6050_read_gyro_z_raw(void);

/**
 * Convert the raw gyro-Z reading to degrees per second.
 * Sensitivity: 131 LSB per (deg/s) at ±250 deg/s full scale.
 */
float mpu6050_get_yaw_rate(void);

#endif /* MPU6050_H */
