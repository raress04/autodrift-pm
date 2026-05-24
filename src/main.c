/* AutoDrift — Main entry point (ATmega328P @ 8 MHz)
 *
 * BLE commands: F/B/L/R/Q/E/D/S, 0-9 (speed level)
 * Status output (1s): CMD:X SPD:NNN YAW:N.NN DRIFT:ON/OFF
 *
 * Wiring: STBY=PB0 PWMA=PB3 PWMB=PB4
 *         AIN1=PD4 AIN2=PD5 BIN1=PD6 BIN2=PD7
 *         HM-10 TX→PD0 RX←PD1
 *         MPU-6050 SCL=PC5 SDA=PC4 AD0=GND
 *
 * Note: disconnect HM-10 RX/TX before flashing.
 *       Car must be stationary during boot calibration (~400ms). */

#include "config.h"
#include "uart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "motors.h"
#include "drift.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

static char    current_cmd   = 'S';
static uint8_t current_speed = 150U;
static float   current_yaw   = 0.0f;

static uint8_t  g_drift_state = 0U; /* 0=IDLE, 1=ACCEL, 2=DRIFT */
static uint16_t g_drift_ticks = 0U;

static void handle_command(char cmd);
static void print_status(void);
static void uart_print_float(float f, uint8_t decimals);

int main(void)
{
    uart_init();
    i2c_init();
    motors_init();
    mpu6050_init();

    uart_puts("AutoDrift boot. Hold still...\r\n");
    mpu6050_calibrate();

    drift_init();
    sei();

    uart_puts("READY. F/B/L/R/Q/E/D/S/G 0-9\r\n");

    uint8_t status_tick = 0;

    while (1) {
        if (uart_available()) {
            char c = uart_getchar();
            if (c != '\r' && c != '\n') handle_command(c);
        }

        if (g_drift_state != 0U) {
            g_drift_ticks++;
            if (g_drift_state == 1U) {
                /* Phase 1: Acceleration - wait 1.2s (60 ticks * 20ms) */
                if (g_drift_ticks >= 60U) {
                    g_drift_state = 2U;
                    g_drift_ticks = 0U;
                    current_cmd = 'Q';
                    current_speed = 224U; /* High speed for initiation */
                    drift_set_base_speed(current_speed);
                    drift_set_target(-60.0f);
                    drift_enable();
                    motors_drift_left(current_speed);
                }
            } else if (g_drift_state == 2U) {
                /* Phase 2: Active Drift Left - drift for 1.6s (80 ticks * 20ms) */
                if (g_drift_ticks >= 80U) {
                    g_drift_state = 0U;
                    g_drift_ticks = 0U;
                    current_cmd = 'S';
                    motors_stop();
                    drift_disable();
                }
            }
        }

        drift_update();
        current_yaw = mpu6050_get_yaw_rate();

        if (++status_tick >= 50U) {
            status_tick = 0;
            print_status();
        }

        _delay_ms(20);
    }

    return 0;
}

static void uart_print_float(float f, uint8_t decimals)
{
    if (f < 0.0f) { uart_putchar('-'); f = -f; }
    int16_t whole = (int16_t)f;
    float   frac  = f - (float)whole;

    char buf[8]; uint8_t i = 0;
    if (whole == 0) {
        uart_putchar('0');
    } else {
        while (whole > 0) { buf[i++] = '0' + (uint8_t)(whole % 10); whole /= 10; }
        while (i--) uart_putchar(buf[i]);
    }

    if (decimals == 0) return;
    uart_putchar('.');
    while (decimals--) {
        frac *= 10.0f;
        uart_putchar('0' + (uint8_t)frac);
        frac -= (uint8_t)frac;
    }
}

static void print_status(void)
{
    uart_puts("CMD:"); uart_putchar(current_cmd);
    uart_puts(" SPD:");
    uint8_t s = current_speed;
    char buf[4]; uint8_t i = 0;
    if (s == 0) { uart_putchar('0'); }
    else { while (s > 0) { buf[i++] = '0' + (s % 10); s /= 10; } while (i--) uart_putchar(buf[i]); }
    uart_puts(" YAW:"); uart_print_float(current_yaw, 2);
    uart_puts(" DRIFT:"); uart_puts(drift_ctrl.enabled ? "ON" : "OFF");
    uart_puts("\r\n");
}

static void handle_command(char cmd)
{
    if (cmd == 'G' || cmd == 'g') {
        g_drift_state = 1U;
        g_drift_ticks = 0U;
        current_cmd = 'G';
        current_speed = 252U; /* Max speed for acceleration */
        drift_set_base_speed(current_speed);
        drift_set_target(0.0f);
        drift_disable();
        motors_forward(current_speed);
        return;
    } else {
        g_drift_state = 0U; /* Cancel automatic sequence */
    }

    current_cmd = cmd;

    switch (cmd) {
        case 'F': case 'f': motors_forward(current_speed);     drift_set_target(0.0f);   break;
        case 'B': case 'b': drift_disable(); motors_backward(current_speed);              break;
        case 'L': case 'l': motors_turn_left(current_speed);   drift_set_target(-15.0f); break;
        case 'R': case 'r': motors_turn_right(current_speed);  drift_set_target(15.0f);  break;
        case 'Q': case 'q': motors_drift_left(current_speed);  drift_set_target(-60.0f); break;
        case 'E': case 'e': motors_drift_right(current_speed); drift_set_target(60.0f);  break;

        case 'D': case 'd':
            if (drift_ctrl.enabled) {
                drift_disable();
                uart_puts("DRIFT:OFF\r\n");
            } else {
                drift_ctrl.integral = 0.0f;
                drift_enable();
                uart_puts("DRIFT:ON\r\n");
            }
            break;

        case 'S': case 's':
            motors_stop();
            drift_disable();
            break;

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9':
            current_speed = (uint8_t)((uint8_t)(cmd - '0') * 28U);
            drift_set_base_speed(current_speed);
            break;

        default: break;
    }
}
