/* AutoDrift — Drift stabilization PI controller */

#include "config.h"
#include "drift.h"
#include "motors.h"
#include "mpu6050.h"

#include <stdint.h>

DriftController drift_ctrl = {
    .enabled    = 0,
    .target_yaw = 0.0f,
    .integral   = 0.0f,
    .base_speed = 150U
};

void drift_init(void)
{
    drift_ctrl.enabled    = 0;
    drift_ctrl.target_yaw = 0.0f;
    drift_ctrl.integral   = 0.0f;
    drift_ctrl.base_speed = 150U;
}

void drift_enable(void)
{
    drift_ctrl.integral = 0.0f;
    drift_ctrl.enabled  = 1;
}

void drift_disable(void)
{
    drift_ctrl.enabled  = 0;
    drift_ctrl.integral = 0.0f;
}

void drift_set_base_speed(uint8_t speed) { drift_ctrl.base_speed = speed; }
void drift_set_target(float target)      { drift_ctrl.target_yaw = target; }

void drift_update(void)
{
    if (!drift_ctrl.enabled) return;

    float yaw   = mpu6050_get_yaw_rate();
    float error = drift_ctrl.target_yaw - yaw;

    /* Suppress noise inside dead zone */
    if (error > -(float)DRIFT_DEAD_ZONE && error < (float)DRIFT_DEAD_ZONE) {
        error = drift_ctrl.integral = 0.0f;
    }

    /* Integrate with anti-windup clamp ±150 */
    drift_ctrl.integral += error * KI_DRIFT;
    if (drift_ctrl.integral >  150.0f) drift_ctrl.integral =  150.0f;
    if (drift_ctrl.integral < -150.0f) drift_ctrl.integral = -150.0f;

    /* Double KP during aggressive drift commands (Q/E) */
    float kp = (drift_ctrl.target_yaw >= 40.0f || drift_ctrl.target_yaw <= -40.0f)
               ? KP_DRIFT * 2.0f : KP_DRIFT;

    float correction = kp * error + drift_ctrl.integral;

    int16_t spd_left  = (int16_t)drift_ctrl.base_speed + (int16_t)correction;
    int16_t spd_right = (int16_t)drift_ctrl.base_speed - (int16_t)correction;

    /* If correction pushes speed negative, actively reverse that motor */
    if (spd_left < 0) {
        int16_t r = (-spd_left > 255) ? 255 : -spd_left;
        motor_left_set(DIR_BACKWARD, (uint8_t)r);
    } else {
        motor_left_set(DIR_FORWARD, (uint8_t)(spd_left > 255 ? 255 : spd_left));
    }

    if (spd_right < 0) {
        int16_t r = (-spd_right > 255) ? 255 : -spd_right;
        motor_right_set(DIR_BACKWARD, (uint8_t)r);
    } else {
        motor_right_set(DIR_FORWARD, (uint8_t)(spd_right > 255 ? 255 : spd_right));
    }
}
