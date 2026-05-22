/* =========================================================
 *  AutoDrift — Bluetooth Motor Test (DEBUG ONLY)
 *  Target: ATmega328P Xplained Mini @ 8 MHz (external clock, mEDBG)
 *
 *  Hardware: LM2596 + TB6612FNG + 2 motors + HM-10 BLE module
 *
 *  Test with LightBlue app:
 *    1. Open LightBlue → scan → connect to "HMSoft" (or your module name)
 *    2. Find the UART characteristic (UUID FFE1)
 *    3. Tap "Write" and send single characters:
 *         'F' → forward (full speed)
 *         'B' → backward (full speed)
 *         'L' → turn left (right motor full, left motor 30%)
 *         'R' → turn right (left motor full, right motor 30%)
 *         'S' → stop (coast)
 *    4. The motors respond immediately.
 *
 *  UART config: 9600 baud, 8N1
 *  HM-10 wiring:
 *    VCC  → 3.3V
 *    GND  → GND
 *    RXD  → PD1 (ATmega TXD)
 *    TXD  → PD0 (ATmega RXD)
 *    BRK  → NC (leave floating)
 *
 *  Build:  pio run -e bttest
 *  Upload: (same avrdude command as motortest)
 * ========================================================= */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* ── Pin definitions ─────────────────────────────────────── */
#define PWMA_PIN   PB3
#define PWMB_PIN   PB4

#define MOTOR_AIN1 PD4
#define MOTOR_AIN2 PD5
#define MOTOR_BIN1 PD6
#define MOTOR_BIN2 PD7

/* STBY (moved from PC5 → PB0 to free I2C SCL) */
#define STBY_PIN   PB0

/* Turn inner wheel ratio: 30% of full speed */
#define INNER_RATIO 30

/* ── UART ────────────────────────────────────────────────── */
#define BAUD     9600UL
/* Board runs at 8 MHz external clock (mEDBG).
 * Normal mode: UBRR = (8000000 / (16 * 9600)) - 1 = 51  (0.2% error) */
#define UBRR_VAL  51

static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = UBRR_VAL;
    /* Enable RX and TX */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* 8 data bits, 1 stop bit, no parity */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static uint8_t uart_available(void)
{
    return (UCSR0A & (1 << RXC0));
}

static char uart_getchar(void)
{
    while (!(UCSR0A & (1 << RXC0)));  /* wait for byte */
    return (char)UDR0;
}

/* Optional: echo back the received char so LightBlue confirms */
static void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putchar(*s++);
}

/* ── Hardware init ───────────────────────────────────────── */
static void hw_init(void)
{
    /* STBY HIGH → enable TB6612FNG (PB0) */
    DDRB  |= (1 << STBY_PIN);
    PORTB |= (1 << STBY_PIN);

    /* PWMA and PWMB as outputs, static HIGH = full speed enable */
    DDRB  |= (1 << PWMA_PIN) | (1 << PWMB_PIN);
    PORTB |= (1 << PWMA_PIN) | (1 << PWMB_PIN);

    /* Direction pins as outputs, all LOW (coast) */
    DDRD  |= (1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
           | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2);
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

/* ── Motor primitives ────────────────────────────────────── */
static void motors_stop(void)
{
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

static void motors_forward(void)
{
    PORTD |=  (1 << MOTOR_AIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN2));
    PORTD |=  (1 << MOTOR_BIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN2));
}

static void motors_backward(void)
{
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN1));
    PORTD |=  (1 << MOTOR_AIN2);
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN1));
    PORTD |=  (1 << MOTOR_BIN2);
}

static void motors_turn_left(void)
{
    /* Left motor stops, right motor runs → car pivots left */
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2))); /* A coast */
    PORTD |=  (1 << MOTOR_BIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN2));
}

static void motors_turn_right(void)
{
    /* Right motor stops, left motor runs → car pivots right */
    PORTD |=  (1 << MOTOR_AIN1);
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN2));
    PORTD &= (uint8_t)(~((1 << MOTOR_BIN1) | (1 << MOTOR_BIN2))); /* B coast */
}

/* ── Command handler ──────────────────────────────────────────── */
static void handle_command(char cmd)
{
    switch (cmd) {
        case 'F': case 'f': motors_forward();    break;
        case 'B': case 'b': motors_backward();   break;
        case 'L': case 'l': motors_turn_left();  break;
        case 'R': case 'r': motors_turn_right(); break;
        case 'S': case 's': motors_stop();       break;
        default: break;  /* ignore unknown / newlines silently */
    }
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    hw_init();
    uart_init();

    /* Announce readiness over BLE — visible in LightBlue console */
    _delay_ms(500);   /* small delay for HM-10 to boot */
    uart_puts("AutoDrift BT ready. F/B/L/R/S\r\n");

    while (1) {
        if (uart_available()) {
            char cmd = uart_getchar();
            handle_command(cmd);
        }
    }

    return 0;
}
