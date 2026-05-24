/* =========================================================
 *  AutoDrift — SSD1306 OLED Stub
 *  OLED removed — output now via BLE UART.
 *  Functions kept as empty stubs to avoid linker errors.
 * ========================================================= */

#include "ssd1306.h"

void ssd1306_init(void)    {}
void ssd1306_clear(void)   {}
void ssd1306_set_cursor(uint8_t x, uint8_t y) { (void)x; (void)y; }
void ssd1306_write_char(char c)   { (void)c; }
void ssd1306_write_string(const char *s) { (void)s; }
void ssd1306_display_status(char cmd, float yaw, uint8_t spd, uint8_t drift_on)
{
    (void)cmd; (void)yaw; (void)spd; (void)drift_on;
}
