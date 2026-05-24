/* =========================================================
 *  AutoDrift — LED Driver Stub
 *  LED removed — PC4/PC5 reserved for I2C on ATmega328P.
 *  Functions kept as empty stubs to avoid linker errors.
 * ========================================================= */

#include "led.h"

void led_init(void)  {}
void led_set(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
void led_off(void)   {}
