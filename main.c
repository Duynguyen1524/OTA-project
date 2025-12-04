#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_flash.h"
#include <stdint.h>
#include <stdio.h>
#include "flash.h"

#define CHUNK_SIZE 256
#define APP_START_ADDR 0x08001000

//---------------- UART Helper ----------------//
void USART1_SendChar(uint8_t c) {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, c);
}

void USART1_SendString(const char *s) {
    while (*s) USART1_SendChar(*s++);
}

void USART1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // PA9 = TX, PA10 = RX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);
}




//---------------- UART Receiver ----------------//
int read_bin_chunk(uint8_t *buffer, uint32_t chunk_size) {
    uint32_t i = 0;
    uint32_t timeout = 1000000; // adjust if needed

    while (i < chunk_size) {
        timeout--;
        if (timeout == 0) break; // no data -> exit early

        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
            buffer[i] = (uint8_t)USART_ReceiveData(USART1);

            // detect stop signal
            if (i > 0 && buffer[i-1] == 0xAA && buffer[i] == 0x55) {
                return -1;
            }
            i++;
        }
    }
    return i; // return actual bytes read
}
//---------------- Main ----------------//
int main(void) {
    USART1_Init();
    USART1_SendString("UART Flash Loader Ready\r\n");

    uint8_t buf[CHUNK_SIZE];
    uint32_t addr = APP_START_ADDR;
    uint32_t chunk_num = 0;

    flash_erase_page(addr);
    USART1_SendString("Flash page erased\r\n");

    while (1) {
        chunk_num++;
        USART1_SendString("Waiting for chunk...\r\n");
        int len = read_bin_chunk(buf, CHUNK_SIZE);

        // Stop signal (0xAA 0x55)
        if (buf[0] == 0xAA && buf[1] == 0x55) {
            USART1_SendString("End signal received\r\n");
            break;
        }

        USART1_SendString("Chunk received: writing to Flash...\r\n");
        flash_program(addr, buf, len);
        USART1_SendString("Write OK\r\n");

        // Verify Flash content
        for (uint32_t i = 0; i < len; i++) {
            if (*(uint8_t *)(addr + i) != buf[i]) {
                USART1_SendString("Verify FAILED\r\n");
                while (1);
            }
        }
        USART1_SendString("Verify OK\r\n");

        addr += len;
    }

    USART1_SendString("Flashing complete\r\n");
    while (1);
}