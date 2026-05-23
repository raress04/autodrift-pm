/* =========================================================
 *  AutoDrift — MPU-6050 Test (DEBUG ONLY)
 *  Target: ATmega328P Xplained Mini @ 8 MHz (external clock)
 *
 *  Hardware present: GY-521 (MPU-6050) + USB serial.
 *  Motors NOT used in this test.
 *
 *  Wiring:
 *    GY-521 VCC → 3.3V
 *    GY-521 GND → GND
 *    GY-521 SCL → PC5 (A5)
 *    GY-521 SDA → PC4 (A4)
 *    GY-521 AD0 → GND  (I2C address = 0x68)
 *    GY-521 INT → NC
 *
 *  Output (USB serial, 9600 8N1):
 *    AX:NNNN AY:NNNN AZ:NNNN GX:NNNN GY:NNNN GZ:NNNN
 *  Raw 16-bit signed values, printed every 500ms.
 *
 *  Build:  pio run -e mpuTest
 *  Monitor: open serial monitor at 9600 baud
 *
 *  NOTE: PC5 is shared with TB6612FNG STBY. Do NOT run
 *        motors and I2C simultaneously without rewiring STBY.
 * ========================================================= */

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>

/* ── UART @ 9600 baud, 8 MHz ─────────────────────────────── */
#define UBRR_9600  51   /* (8000000/(16*9600)) - 1 */

static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = UBRR_9600;
    UCSR0B = (1 << TXEN0);               /* TX only for this test */
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putchar(*s++);
}

/* Print signed 16-bit integer */
static void uart_print_int16(int16_t v)
{
    char buf[8];
    uint8_t i = 0;
    uint8_t neg = 0;

    if (v < 0) { neg = 1; v = -v; }
    if (v == 0) { uart_putchar('0'); return; }

    while (v > 0) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    if (neg) buf[i++] = '-';

    while (i--) uart_putchar(buf[i]);
}

/* ── I2C / TWI @ 100 kHz, 8 MHz ─────────────────────────── */
/* TWBR = ((F_CPU/SCL) - 16) / (2 * prescaler)
 *       = ((8000000/100000) - 16) / 2 = 32               */
#define TWI_TWBR  32

static void i2c_init(void)
{
    TWSR = 0x00;   /* prescaler = 1 */
    TWBR = TWI_TWBR;
    TWCR = (1 << TWEN);
}

static uint8_t i2c_start(uint8_t addr_rw)
{
    /* START condition */
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    if ((TWSR & 0xF8) != TW_START && (TWSR & 0xF8) != TW_REP_START)
        return 1;

    /* Send address + R/W */
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    uint8_t status = TWSR & 0xF8;
    if (status != TW_MT_SLA_ACK && status != TW_MR_SLA_ACK)
        return 1;

    return 0;
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

static uint8_t i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
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
#define MPU_ADDR      0x68   /* AD0 = GND → address 0x68 */
#define MPU_PWR_MGMT  0x6B
#define MPU_ACCEL_OUT 0x3B   /* first of 6 accel bytes */
#define MPU_GYRO_OUT  0x43   /* first of 6 gyro  bytes */

/* Write a single register */
static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    if (i2c_start((MPU_ADDR << 1) | TW_WRITE)) return 1;
    if (i2c_write(reg))  { i2c_stop(); return 1; }
    if (i2c_write(val))  { i2c_stop(); return 1; }
    i2c_stop();
    return 0;
}

/* Read 6 consecutive bytes starting at reg into buf */
static uint8_t mpu_read6(uint8_t reg, int16_t *buf)
{
    uint8_t raw[6];

    if (i2c_start((MPU_ADDR << 1) | TW_WRITE)) return 1;
    if (i2c_write(reg)) { i2c_stop(); return 1; }

    if (i2c_start((MPU_ADDR << 1) | TW_READ)) return 1;
    raw[0] = i2c_read_ack();
    raw[1] = i2c_read_ack();
    raw[2] = i2c_read_ack();
    raw[3] = i2c_read_ack();
    raw[4] = i2c_read_ack();
    raw[5] = i2c_read_nack();
    i2c_stop();

    buf[0] = (int16_t)((raw[0] << 8) | raw[1]);
    buf[1] = (int16_t)((raw[2] << 8) | raw[3]);
    buf[2] = (int16_t)((raw[4] << 8) | raw[5]);
    return 0;
}

static uint8_t mpu_init(void)
{
    _delay_ms(100);   /* wait for MPU power-on */
    /* Clear SLEEP bit → wake up MPU */
    return mpu_write_reg(MPU_PWR_MGMT, 0x00);
}

/* ── Main ────────────────────────────────────────────────── */
int main(void)
{
    uart_init();
    i2c_init();

    _delay_ms(200);
    uart_puts("MPU-6050 test starting...\r\n");

    if (mpu_init()) {
        uart_puts("ERROR: MPU-6050 not found! Check wiring.\r\n");
        while (1);   /* halt */
    }

    uart_puts("OK: MPU-6050 found. Reading every 500ms.\r\n");
    uart_puts("Format: AX AY AZ GX GY GZ (raw 16-bit signed)\r\n\r\n");

    int16_t accel[3], gyro[3];

    while (1) {
        if (mpu_read6(MPU_ACCEL_OUT, accel) || mpu_read6(MPU_GYRO_OUT, gyro)) {
            uart_puts("READ ERROR\r\n");
        } else {
            uart_puts("AX:"); uart_print_int16(accel[0]);
            uart_puts(" AY:"); uart_print_int16(accel[1]);
            uart_puts(" AZ:"); uart_print_int16(accel[2]);
            uart_puts(" | GX:"); uart_print_int16(gyro[0]);
            uart_puts(" GY:"); uart_print_int16(gyro[1]);
            uart_puts(" GZ:"); uart_print_int16(gyro[2]);
            uart_puts("\r\n");
        }
        _delay_ms(500);
    }

    return 0;
}
