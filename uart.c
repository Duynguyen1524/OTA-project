#include "stm32f10x.h"
#include "uart.h"

static volatile uint32_t g_ms = 0;

void tim2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 72-1;        // 72 MHz / 72 = 1 MHz
    TIM2->ARR = 1000-1;      // 1 MHz / 1000 = 1 kHz (1 ms)
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        g_ms++;
    }
}

uint32_t millis(void) { return g_ms; }

/* 72MHz SYSCLK, APB1=36, APB2=72 */
void SystemClock72MHz(void){
    RCC->CR |= RCC_CR_HSEON; while(!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR = 0;
    RCC->CFGR |= (4<<8);                  /* APB1 /2 -> 36MHz */
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR   |= RCC_CR_PLLON; while(!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* -------- USART1 with RX ring buffer ---------- */
#define RB_SZ 256u
static volatile uint8_t rb[RB_SZ];
static volatile uint16_t r_head, r_tail;

void USART1_IRQHandler(void){
    if (USART1->SR & USART_SR_RXNE){
        rb[r_head] = (uint8_t)USART1->DR;
        r_head = (uint16_t)((r_head + 1) & (RB_SZ-1));
    }
}

static void usart1_set_baud(uint32_t pclk, uint32_t baud){
    uint32_t div16 = (pclk + baud/2u) / baud;
    USART1->BRR = ((div16/16u)<<4) | (div16%16u);
}

void uart1_init(uint32_t baud){
    r_head = r_tail = 0;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;        /* PA9/PA10 */

    /* PA9 TX AF-PP 50MHz, PA10 RX input PU */
    GPIOA->CRH &= ~((0xF<<(4*1))|(0xF<<(4*2)));
    GPIOA->CRH |=  (0xB<<(4*1)) | (0x8<<(4*2));
    GPIOA->ODR |= (1<<10);

    USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE;
    usart1_set_baud(72000000u, baud);
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

    NVIC_EnableIRQ(USART1_IRQn);
}

static void uart_putc(uint8_t c) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {}
	USART_SendData(USART1, c);
}





static uint8_t uart_getc_blocking(void) {
while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET) {}
return (uint8_t)USART_ReceiveData(USART1);
}


static int uart_read_exact(uint8_t* buf, uint32_t len) {
for (uint32_t i = 0; i < len; i++) buf[i] = uart_getc_blocking();
return 0;
}


