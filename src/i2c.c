/* AutoDrift — I2C (TWI) master driver (ATmega328P, 100 kHz) */

#include "config.h"
#include "i2c.h"

#include <avr/io.h>
#include <util/twi.h>

/* Spin-wait for TWINT with a timeout to avoid bus lockup. */
static inline void twi_wait(void)
{
    uint16_t t = 10000U;
    while (!(TWCR & (1 << TWINT)) && --t)
        ;
}

void i2c_init(void)
{
    TWBR = TWBR_VALUE;
    TWSR = 0x00;            /* prescaler = 1 */
    TWCR = (1 << TWEN);
}

uint8_t i2c_start(uint8_t address, uint8_t rw)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    twi_wait();

    TWDR = (uint8_t)((address << 1) | rw);
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();

    return (uint8_t)(TWSR & 0xF8);
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t i2c_write_byte(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    return (uint8_t)(TWSR & 0xF8);
}

uint8_t i2c_read_byte_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    twi_wait();
    return TWDR;
}

uint8_t i2c_read_byte_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    twi_wait();
    return TWDR;
}

uint8_t i2c_write_register(uint8_t dev, uint8_t reg, uint8_t val)
{
    if (i2c_start(dev, TW_WRITE) != TW_MT_SLA_ACK) {
        i2c_stop();
        return 1;
    }
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_stop();
    return 0;
}

uint8_t i2c_read_register(uint8_t dev, uint8_t reg)
{
    uint8_t value = 0;
    if (i2c_start(dev, TW_WRITE) != TW_MT_SLA_ACK) { i2c_stop(); return 0; }
    i2c_write_byte(reg);
    if (i2c_start(dev, TW_READ)  != TW_MR_SLA_ACK) { i2c_stop(); return 0; }
    value = i2c_read_byte_nack();
    i2c_stop();
    return value;
}

void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return;
    if (i2c_start(dev, TW_WRITE) != TW_MT_SLA_ACK) { i2c_stop(); return; }
    i2c_write_byte(reg);
    if (i2c_start(dev, TW_READ)  != TW_MR_SLA_ACK) { i2c_stop(); return; }

    for (uint8_t i = 0; i < len; i++) {
        buf[i] = (i < (uint8_t)(len - 1U))
                     ? i2c_read_byte_ack()
                     : i2c_read_byte_nack();
    }
    i2c_stop();
}
