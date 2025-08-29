#ifndef UART_H
#define UART_H
#include <stdint.h>
#include "stm32f10x_usart.h"

#define MAX_LINE            256
#define UARTx               USART1       // using PA9 (TX) / PA10 (RX)
#define UART_BAUD           115200
void     SystemClock72MHz(void);          /* 8MHz HSE -> 72MHz SYSCLK */
void     tim2_init(void);
uint32_t millis(void);

/* USART1 on PA9/PA10 */
void uart1_init(uint32_t baud);
static void uart_putc(uint8_t c) ;



static uint8_t uart_getc_blocking(void);
static int uart_getline(char *buf, int maxlen) {
    int n = 0;
    while (n < maxlen - 1) {
        int c = uart_getc_blocking();
        if (c == '\r') {
            // normalize CRLF
            buf[n++] = '\n';
            break;
        }
        buf[n++] = (char)c;
        if (c == '\n') break;
    }
    buf[n] = 0;
    return n;
}

static int uart_read_exact(uint8_t* buf, uint32_t len);

static void uart_write(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}
#endif
