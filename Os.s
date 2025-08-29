    AREA |.TEXT|, CODE, READONLY, ALIGN=2
    THUMB
    EXTERN currentPt

    EXPORT osSchedulerLaunch
    EXPORT SysTick_Handler

;--------------------------------------
; Start the first thread
;--------------------------------------
osSchedulerLaunch
    LDR R0, =currentPt
	LDR R2, [R0]
	
	LDR SP, [R2]
	POP {R4-R11}
	POP {R0-R3}
	POP {R12}
	ADD SP, SP, #4
	POP {LR}
	ADD SP, SP, #4
	
	CPSIE I
	BX LR
                 ; Jump to thread's PC (usually via exception return)

;--------------------------------------
; SysTick interrupt handler for context switch
;--------------------------------------
SysTick_Handler
    CPSID I
	PUSH {R4-R11}
	LDR R0, =currentPt
	LDR R1, [R0]
	
	STR SP, [R1]
	LDR R1, [R1, #4]
	STR R1, [R0]
	
	LDR SP, [R1]
	POP {R4-R11}
	CPSIE I
	BX LR
                ; Return from SysTick ISR
