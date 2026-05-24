/* AutoDrift — UART0 driver (ATmega328P, 9600 8N1)
 * ISR fills ring buffer; main loop consumes via uart_getchar(). */

#include "config.h"
#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>

static volatile char    rx_buffer[UART_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

#define RX_MASK  ((uint8_t)(UART_BUFFER_SIZE - 1U))

ISR(USART_RX_vect)
{
    char c = UDR0;
    uint8_t next = (rx_head + 1U) & RX_MASK;
    if (next != rx_tail) {          /* drop if full */
        rx_buffer[rx_head] = c;
        rx_head = next;
    }
}

void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8N1 */
}

uint8_t uart_available(void)
{
    return (uint8_t)((rx_head - rx_tail) & RX_MASK);
}

char uart_getchar(void)
{
    while (uart_available() == 0)
        ;
    char c = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1U) & RX_MASK;
    return c;
}

void uart_putchar(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = (uint8_t)c;
}

void uart_puts(const char *s)
{
    while (*s) uart_putchar(*s++);
}
