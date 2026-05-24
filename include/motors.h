#ifndef MOTORS_H
#define MOTORS_H

/* =========================================================
 *  AutoDrift — Motor Control Driver
 *  Target: ATmega324P @ 16 MHz
 *
 *  Timer0 Fast PWM drives both motors:
 *    OC0A (PB3) = Motor A (LEFT  rear) via L298N ENA
 *    OC0B (PB4) = Motor B (RIGHT rear) via L298N ENB
 *
 *  Direction via PORT D:
 *    IN1 (PD2) / IN2 (PD3) = Motor A direction
 *    IN3 (PD4) / IN4 (PD5) = Motor B direction
 *
 *  NO SERVO. NO TIMER1. Differential speed = steering.
 * ========================================================= */

#include <stdint.h>

/* ── Direction enum ─────────────────────────────────────── */
typedef enum {
    DIR_FORWARD  = 0, /**< IN1=H IN2=L — motor spins forward   */
    DIR_BACKWARD = 1, /**< IN1=L IN2=H — motor spins backward  */
    DIR_BRAKE    = 2, /**< IN1=H IN2=H — motor actively braked */
    DIR_COAST    = 3  /**< IN1=L IN2=L — motor coasts to stop  */
} MotorDir;

/* ── Initialisation ─────────────────────────────────────── */

/**
 * Configure Timer0 (Fast PWM, ~7.8 kHz) and direction GPIO.
 * Call once before sei().
 */
void motors_init(void);

/* ── Per-motor primitives ───────────────────────────────── */

/**
 * Set Motor A (LEFT rear wheel) direction and PWM speed.
 * @param dir   One of DIR_FORWARD, DIR_BACKWARD, DIR_BRAKE, DIR_COAST.
 * @param speed PWM duty cycle 0–255 (ignored for BRAKE and COAST).
 */
void motor_left_set(MotorDir dir, uint8_t speed);

/**
 * Set Motor B (RIGHT rear wheel) direction and PWM speed.
 * @param dir   One of DIR_FORWARD, DIR_BACKWARD, DIR_BRAKE, DIR_COAST.
 * @param speed PWM duty cycle 0–255 (ignored for BRAKE and COAST).
 */
void motor_right_set(MotorDir dir, uint8_t speed);

/* ── High-level manoeuvres ──────────────────────────────── */

/** Both motors forward at equal speed — drives straight. */
void motors_forward(uint8_t speed);

/** Both motors backward at equal speed. */
void motors_backward(uint8_t speed);

/** Differential turn left: left motor 30 %, right motor 100 %. */
void motors_turn_left(uint8_t speed);

/** Differential turn right: left motor 100 %, right motor 30 %. */
void motors_turn_right(uint8_t speed);

/** Hard drift left: left motor off, right motor full speed (rear kicks left). */
void motors_drift_left(uint8_t speed);

/** Hard drift right: right motor off, left motor full speed (rear kicks right). */
void motors_drift_right(uint8_t speed);

/** Coast both motors to stop (IN1=IN2=IN3=IN4=0). */
void motors_stop(void);

/** Actively brake both motors (IN1=IN2=IN3=IN4=1, PWM=0). */
void motors_brake(void);

#endif /* MOTORS_H */
