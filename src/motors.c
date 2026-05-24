/* AutoDrift — TB6612FNG motor driver (ATmega328P)
 * Software PWM via Timer2 overflow @ 31.25 kHz (8MHz/256). */

#include "config.h"
#include "motors.h"

#include <avr/io.h>
#include <avr/interrupt.h>

static volatile uint8_t pwm_left_val  = 0;
static volatile uint8_t pwm_right_val = 0;

ISR(TIMER2_OVF_vect)
{
    static uint8_t count = 0;
    count++;
    if (count < pwm_left_val)  PORTB |=  (1 << PB3);
    else                       PORTB &= ~(1 << PB3);
    if (count < pwm_right_val) PORTB |=  (1 << PB4);
    else                       PORTB &= ~(1 << PB4);
}

void motors_init(void)
{
    uint8_t sreg = SREG;
    cli();

    /* STBY high → enable driver */
    STBY_DDR  |= (1 << STBY_PIN);
    STBY_PORT |= (1 << STBY_PIN);

    /* PWM pins as outputs, low initially */
    PWMA_DDR |= (1 << PWMA_PIN);
    PWMB_DDR |= (1 << PWMB_PIN);
    PORTB &= ~((1 << PWMA_PIN) | (1 << PWMB_PIN));

    /* Direction pins as outputs, all low (coast) */
    AIN_DDR |= (1 << MOTOR_AIN1) | (1 << MOTOR_AIN2);
    BIN_DDR |= (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2);
    AIN_PORT &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)));
    BIN_PORT &= (uint8_t)(~((1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));

    /* Timer2 normal mode, no prescaler → 31.25 kHz overflow */
    TCCR2A = 0x00;
    TCCR2B = (1 << CS20);
    TIMSK2 |= (1 << TOIE2);

    SREG = sreg;
}

void motor_left_set(MotorDir dir, uint8_t speed)
{
    uint8_t port = AIN_PORT & (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)));

    switch (dir) {
        case DIR_FORWARD:  port |= (1 << MOTOR_AIN1); pwm_left_val = speed;  break;
        case DIR_BACKWARD: port |= (1 << MOTOR_AIN2); pwm_left_val = speed;  break;
        case DIR_BRAKE:    port |= (1 << MOTOR_AIN1) | (1 << MOTOR_AIN2); pwm_left_val = 255; break;
        case DIR_COAST:
        default:           pwm_left_val = 0; break;
    }
    AIN_PORT = port;
}

void motor_right_set(MotorDir dir, uint8_t speed)
{
    uint8_t port = BIN_PORT & (uint8_t)(~((1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));

    switch (dir) {
        case DIR_FORWARD:  port |= (1 << MOTOR_BIN1); pwm_right_val = speed; break;
        case DIR_BACKWARD: port |= (1 << MOTOR_BIN2); pwm_right_val = speed; break;
        case DIR_BRAKE:    port |= (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2); pwm_right_val = 255; break;
        case DIR_COAST:
        default:           pwm_right_val = 0; break;
    }
    BIN_PORT = port;
}

void motors_forward(uint8_t speed)  { motor_left_set(DIR_FORWARD,  speed); motor_right_set(DIR_FORWARD,  speed); }
void motors_backward(uint8_t speed) { motor_left_set(DIR_BACKWARD, speed); motor_right_set(DIR_BACKWARD, speed); }

void motors_turn_left(uint8_t speed)
{
    uint8_t inner = (uint8_t)(((uint16_t)speed * TURN_INNER_RATIO) / 100U);
    motor_left_set(DIR_FORWARD, inner);
    motor_right_set(DIR_FORWARD, speed);
}

void motors_turn_right(uint8_t speed)
{
    uint8_t inner = (uint8_t)(((uint16_t)speed * TURN_INNER_RATIO) / 100U);
    motor_left_set(DIR_FORWARD, speed);
    motor_right_set(DIR_FORWARD, inner);
}

void motors_drift_left(uint8_t speed)  { motor_left_set(DIR_COAST,   0); motor_right_set(DIR_FORWARD, speed); }
void motors_drift_right(uint8_t speed) { motor_left_set(DIR_FORWARD, speed); motor_right_set(DIR_COAST, 0);   }

void motors_stop(void)  { motor_left_set(DIR_COAST, 0); motor_right_set(DIR_COAST, 0); }
void motors_brake(void) { motor_left_set(DIR_BRAKE, 0); motor_right_set(DIR_BRAKE, 0); }
