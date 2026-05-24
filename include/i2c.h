#ifndef I2C_H
#define I2C_H

/* =========================================================
 *  AutoDrift — I2C (TWI) Master Driver
 *  Target: ATmega324P @ 16 MHz, 400 kHz Fast Mode
 * ========================================================= */

#include <stdint.h>

/* ── Public API ──────────────────────────────────────────── */

/** Initialise TWI peripheral at 400 kHz. */
void i2c_init(void);

/**
 * Send START + SLA+R/W.
 * @param address  7-bit device address.
 * @param rw       0 = write, 1 = read.
 * @return TW_STATUS code (see <util/twi.h>).
 */
uint8_t i2c_start(uint8_t address, uint8_t rw);

/** Send STOP condition and release the bus. */
void i2c_stop(void);

/**
 * Write one byte and wait for ACK.
 * @return TW_STATUS code.
 */
uint8_t i2c_write_byte(uint8_t data);

/** Read one byte and ACK the slave (more bytes to follow). */
uint8_t i2c_read_byte_ack(void);

/** Read one byte and NACK the slave (last byte). */
uint8_t i2c_read_byte_nack(void);

/* ── Convenience helpers ─────────────────────────────────── */

/**
 * Write a single register.
 * @return 0 on success, non-zero on bus error.
 */
uint8_t i2c_write_register(uint8_t dev, uint8_t reg, uint8_t val);

/**
 * Read a single register.
 * @return Register value (0 on bus error — caller cannot distinguish).
 */
uint8_t i2c_read_register(uint8_t dev, uint8_t reg);

/**
 * Burst-read @p len consecutive registers starting at @p reg.
 * Result written into @p buf.
 */
void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* I2C_H */
