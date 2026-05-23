/* =========================================================
 *  AutoDrift — Combined BLE Motor + MPU-6050 Test (DEBUG)
 *  Target: ATmega328P Xplained Mini @ 8 MHz (external clock)
 *
 *  Hardware: LM2596 + TB6612FNG + 2 motors + HM-10 + GY-521
 *
 *  BLE commands (send from app, UTF-8):
 *    'F' → forward       'B' → backward
 *    'L' → turn left     'R' → turn right
 *    'S' → stop
 *
 *  BLE output (subscribe/notify on FFE1):
 *    AX:NNN AY:NNN AZ:NNN | GX:NNN GY:NNN GZ:NNN
 *    Streamed every 500ms.
 *
 *  Pin mapping:
 *    STBY → PB0          (TB6612FNG enable)
 *    PWMA → PB3          PWMB → PB4
 *    AIN1 → PD4          AIN2 → PD5
 *    BIN1 → PD6          BIN2 → PD7
 *    HM-10 TXD → PD0    HM-10 RXD → PD1
 *    GY-521 SCL → PC5   GY-521 SDA → PC4
 *    GY-521 AD0 → GND   (address 0x68)
 *
 *  Build:   pio run -e fulltest
 *  Upload:  standard avrdude via WSL
 * ========================================================= */

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

/* ── UART @ 9600 baud, 8 MHz ─────────────────────────────── */
#define UBRR_9600  51

static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = UBRR_9600;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static uint8_t uart_available(void) { return UCSR0A & (1 << RXC0); }
static char    uart_getchar(void)   { while (!uart_available()); return (char)UDR0; }

static void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putchar(*s++); }

static void uart_print_int16(int16_t v)
{
    char buf[8]; uint8_t i = 0;
    if (v < 0) { uart_putchar('-'); v = -v; }
    if (v == 0) { uart_putchar('0'); return; }
    while (v > 0) { buf[i++] = '0' + (uint8_t)(v % 10); v /= 10; }
    while (i--) uart_putchar(buf[i]);
}

/* ── I2C / TWI @ 100 kHz, 8 MHz ─────────────────────────── */
static void i2c_init(void)
{
    TWSR = 0x00;
    TWBR = 32;   /* (8000000/100000 - 16) / 2 */
    TWCR = (1 << TWEN);
}

static uint8_t i2c_start(uint8_t addr_rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_START && (TWSR & 0xF8) != TW_REP_START) return 1;
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    uint8_t s = TWSR & 0xF8;
    return (s != TW_MT_SLA_ACK && s != TW_MR_SLA_ACK) ? 1 : 0;
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

static uint8_t i2c_write(uint8_t d)
{
    TWDR = d; TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return ((TWSR & 0xF8) != TW_MT_DATA_ACK);
}

static uint8_t i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static uint8_t i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

/* ── MPU-6050 ────────────────────────────────────────────── */
#define MPU_ADDR      0x68
#define MPU_PWR_MGMT  0x6B
#define MPU_ACCEL_OUT 0x3B
#define MPU_GYRO_OUT  0x43

static uint8_t mpu_init(void)
{
    _delay_ms(100);
    if (i2c_start((MPU_ADDR << 1) | TW_WRITE)) return 1;
    if (i2c_write(MPU_PWR_MGMT)) { i2c_stop(); return 1; }
    if (i2c_write(0x00))         { i2c_stop(); return 1; }
    i2c_stop();
    return 0;
}

static uint8_t mpu_read6(uint8_t reg, int16_t *buf)
{
    uint8_t raw[6];
    if (i2c_start((MPU_ADDR << 1) | TW_WRITE)) return 1;
    if (i2c_write(reg)) { i2c_stop(); return 1; }
    if (i2c_start((MPU_ADDR << 1) | TW_READ))  return 1;
    raw[0] = i2c_read_ack(); raw[1] = i2c_read_ack();
    raw[2] = i2c_read_ack(); raw[3] = i2c_read_ack();
    raw[4] = i2c_read_ack(); raw[5] = i2c_read_nack();
    i2c_stop();
    buf[0] = (int16_t)((raw[0] << 8) | raw[1]);
    buf[1] = (int16_t)((raw[2] << 8) | raw[3]);
    buf[2] = (int16_t)((raw[4] << 8) | raw[5]);
    return 0;
}

/* ── Motors ──────────────────────────────────────────────── */
#define STBY_PIN   PB0
#define PWMA_PIN   PB3
#define PWMB_PIN   PB4
#define MOTOR_AIN1 PD4
#define MOTOR_AIN2 PD5
#define MOTOR_BIN1 PD6
#define MOTOR_BIN2 PD7

static void motors_init(void)
{
    DDRB  |= (1 << STBY_PIN) | (1 << PWMA_PIN) | (1 << PWMB_PIN);
    PORTB |= (1 << STBY_PIN) | (1 << PWMA_PIN) | (1 << PWMB_PIN);
    DDRD  |= (1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
           | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2);
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

static void motors_stop(void)
{
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)
                        | (1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

static void motors_forward(void)
{
    PORTD |=  (1 << MOTOR_AIN1); PORTD &= (uint8_t)(~(1 << MOTOR_AIN2));
    PORTD |=  (1 << MOTOR_BIN1); PORTD &= (uint8_t)(~(1 << MOTOR_BIN2));
}

static void motors_backward(void)
{
    PORTD &= (uint8_t)(~(1 << MOTOR_AIN1)); PORTD |= (1 << MOTOR_AIN2);
    PORTD &= (uint8_t)(~(1 << MOTOR_BIN1)); PORTD |= (1 << MOTOR_BIN2);
}

static void motors_turn_left(void)
{
    PORTD &= (uint8_t)(~((1 << MOTOR_AIN1) | (1 << MOTOR_AIN2)));
    PORTD |=  (1 << MOTOR_BIN1); PORTD &= (uint8_t)(~(1 << MOTOR_BIN2));
}

static void motors_turn_right(void)
{
    PORTD |=  (1 << MOTOR_AIN1); PORTD &= (uint8_t)(~(1 << MOTOR_AIN2));
    PORTD &= (uint8_t)(~((1 << MOTOR_BIN1) | (1 << MOTOR_BIN2)));
}

/* ── Command handler ─────────────────────────────────────── */
static void handle_cmd(char cmd)
{
    switch (cmd) {
        case 'F': case 'f': motors_forward();    break;
        case 'B': case 'b': motors_backward();   break;
        case 'L': case 'l': motors_turn_left();  break;
        case 'R': case 'r': motors_turn_right(); break;
        case 'S': case 's': motors_stop();       break;
        default: break;
    }
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    uart_init();
    i2c_init();
    motors_init();

    _delay_ms(500);   /* HM-10 boot time */

    if (mpu_init()) {
        uart_puts("ERR:MPU\r\n");
        while (1);
    }
    uart_puts("OK:ready F/B/L/R/S\r\n");

    int16_t accel[3], gyro[3];
    uint8_t tick = 0;   /* counts 50ms loops; at 10 → 500ms = send MPU data */

    while (1) {
        /* ── Check for incoming BLE command ─────────────── */
        if (uart_available()) {
            handle_cmd(uart_getchar());
        }

        /* ── Every 500ms: read MPU and stream data ──────── */
        _delay_ms(50);
        if (++tick >= 10) {
            tick = 0;
            if (!mpu_read6(MPU_ACCEL_OUT, accel) &&
                !mpu_read6(MPU_GYRO_OUT,  gyro))
            {
                uart_puts("AX:"); uart_print_int16(accel[0]);
                uart_puts(" AY:"); uart_print_int16(accel[1]);
                uart_puts(" AZ:"); uart_print_int16(accel[2]);
                uart_puts("|GX:"); uart_print_int16(gyro[0]);
                uart_puts(" GY:"); uart_print_int16(gyro[1]);
                uart_puts(" GZ:"); uart_print_int16(gyro[2]);
                uart_puts("\r\n");
            }
        }
    }

    return 0;
}
