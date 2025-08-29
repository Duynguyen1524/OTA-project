#ifndef __OS_KERNEL_H
#define __OS_KERNEL_H

#include <stdint.h>

// ---------- Kernel Configuration ----------
#define NUM_OF_THREADS     3
#define STACK_SIZE         100
#define BUS_FREQ           72000000

// ---------- SysTick Control Macros ----------
#define CTRL_ENABLE        (1 << 0)
#define CTRL_TICKINT       (1 << 1)
#define CTRL_CLCKSRC       (1 << 2)
#define CTRL_COUNTFLAG     (1 << 16)
#define SYSTICK_RST        0
#define INTCTRL (*((volatile uint32_t *)0XE000ED04))
#define PENDSTSET   (1<<26)


// ---------- Typedefs ----------
typedef struct tcb {
    uint32_t *stackPt;         // Stack pointer
    struct tcb *nextPt;        // Pointer to next TCB
} tcbType;

// ---------- External Variables ----------
extern uint32_t MILLIS_PRESCALER;
extern tcbType *currentPt;

// ---------- Kernel API ----------
void osKernelInit(void);
void osKernelStackInit(int i);
uint8_t osKernelAddThread(void(*task0)(void), void(*task1)(void));
void osKernelLaunch(uint32_t quanta);
extern void osSchedulerLaunch(void);   // Implemented in .s file
void osThreadYield();
void osSemaphore_Init(uint32_t *semaphore, uint32_t val);
void osSemaphore_Give(uint32_t *semaphore);
void osSpinLock_Wait(uint32_t *semaphore);
void osCooperative_Wait(uint32_t *semaphore);
#endif  // __OS_KERNEL_H
