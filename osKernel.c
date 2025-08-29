#include "osKernel.h"
#include "stm32f10x.h"

tcbType tcbs[NUM_OF_THREADS];
tcbType *currentPt;
uint32_t TCB_STACK[NUM_OF_THREADS][STACK_SIZE];
uint32_t MILLIS_PRESCALER;
extern void osSchedulerLaunch(void);
__attribute__((used)) tcbType *currentPt;


void osKernelInit(void){
	//1ms
	MILLIS_PRESCALER = (BUS_FREQ/1000);
}

void osKernelStackInit(int i){
    tcbs[i].stackPt = &TCB_STACK[i][STACK_SIZE-16];

    TCB_STACK[i][STACK_SIZE-1]  = 1U << 24;        // PSR (Thumb bit set)
    TCB_STACK[i][STACK_SIZE-2]  = 0xAAAAAAAA;      // PC (set in AddThread)
    TCB_STACK[i][STACK_SIZE-3]  = 0xAAAAAAAA;      // LR
    TCB_STACK[i][STACK_SIZE-4]  = 0xAAAAAAAA;      // R12
    TCB_STACK[i][STACK_SIZE-5]  = 0xAAAAAAAA;      // R3
    TCB_STACK[i][STACK_SIZE-6]  = 0xAAAAAAAA;      // R2
    TCB_STACK[i][STACK_SIZE-7]  = 0xAAAAAAAA;      // R1
    TCB_STACK[i][STACK_SIZE-8]  = 0xAAAAAAAA;      // R0

    TCB_STACK[i][STACK_SIZE-9]  = 0xAAAAAAAA;      // R11
    TCB_STACK[i][STACK_SIZE-10] = 0xAAAAAAAA;      // R10
    TCB_STACK[i][STACK_SIZE-11] = 0xAAAAAAAA;      // R9
    TCB_STACK[i][STACK_SIZE-12] = 0xAAAAAAAA;      // R8
    TCB_STACK[i][STACK_SIZE-13] = 0xAAAAAAAA;      // R7
    TCB_STACK[i][STACK_SIZE-14] = 0xAAAAAAAA;      // R6
    TCB_STACK[i][STACK_SIZE-15] = 0xAAAAAAAA;      // R5
    TCB_STACK[i][STACK_SIZE-16] = 0xAAAAAAAA;      // R4
}

uint8_t osKernelAddThread(void (*task0)(void), void (*task1)(void)){
    __disable_irq();

    tcbs[0].nextPt = &tcbs[1];
    tcbs[1].nextPt = &tcbs[0];
   

    osKernelStackInit(0);
    TCB_STACK[0][STACK_SIZE-2] = (uint32_t)(task0); // Set PC

    osKernelStackInit(1);
    TCB_STACK[1][STACK_SIZE-2] = (uint32_t)(task1);


    currentPt = &tcbs[0];

    __enable_irq();

    return 1;
}
void osThreadYield(){
	SysTick->VAL = 0;
	INTCTRL |= PENDSTSET ;
}
void osSemaphore_Init(uint32_t *semaphore, uint32_t val){
	*semaphore = val;
}
void osSemaphore_Give(uint32_t *semaphore){
	__disable_irq();
	*semaphore +=1;
	__enable_irq();
}
void osSpinLock_Wait(uint32_t *semaphore){
	__disable_irq();
	
	while(*semaphore <= 0){
		__enable_irq();
		__disable_irq();
	}
	*semaphore -=1;
	
	__enable_irq();
}
void osCooperative_Wait(uint32_t *semaphore){
	__disable_irq();
	
	while(*semaphore <= 0){
		__enable_irq();
		osThreadYield();
		__disable_irq();
	}
	*semaphore -=1;
	
	__enable_irq();
}


void osKernelLaunch(uint32_t quanta){
	/*Reset the SysTick*/
	SysTick->CTRL = SYSTICK_RST;
	/*Clear Systick current value register*/
	SysTick->VAL = 0;
	/*Load the quanta*/
	SysTick->LOAD = (quanta*MILLIS_PRESCALER-1);
	/*Set Systick interupt to lowest priority*/
	NVIC_SetPriority(SysTick_IRQn, 15);
	/*Enable Systick, select Internal clock*/
	SysTick->CTRL |= CTRL_CLCKSRC|CTRL_ENABLE;
	/*Enable Systick interupt*/
	SysTick->CTRL |= CTRL_TICKINT;
	//os launch scheduler
	osSchedulerLaunch();
}

