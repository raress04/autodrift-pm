#ifndef DRIFT_H
#define DRIFT_H

/* =========================================================
 *  AutoDrift — Drift Stabilization Controller
 *  Target: ATmega324P @ 16 MHz
 *
 *  When enabled, a PI loop reads the gyro Z-axis (yaw rate)
 *  and applies a differential PWM correction to keep the car
 *  at a commanded yaw rate instead of spinning out.
 *
 *  Call drift_update() every 20 ms from the main loop.
 *  Do NOT call from an ISR — uses floating-point arithmetic.
 * ========================================================= */

#include <stdint.h>

/* ── Controller state ───────────────────────────────────── */
typedef struct {
    uint8_t enabled;      /**< 0 = off, 1 = on                   */
    float   target_yaw;   /**< Desired yaw rate [deg/s]           */
    float   integral;     /**< PI integral accumulator            */
    uint8_t base_speed;   /**< Base PWM for both motors (0–255)   */
} DriftController;

/** Global controller instance — readable by main.c for display. */
extern DriftController drift_ctrl;

/* ── API ─────────────────────────────────────────────────── */

/** Initialise controller with defaults (disabled, base speed 150). */
void drift_init(void);

/** Enable the PI controller and reset the integral. */
void drift_enable(void);

/** Disable the PI controller and reset the integral. */
void drift_disable(void);

/** Update the base PWM speed used by the controller. */
void drift_set_base_speed(uint8_t speed);

/** Set the desired yaw rate [deg/s] the controller will track. */
void drift_set_target(float target_yaw_rate);

/**
 * Execute one PI correction step.
 * Must be called every 20 ms.
 * Does nothing when drift_ctrl.enabled == 0.
 */
void drift_update(void);

#endif /* DRIFT_H */
