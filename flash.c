#include "stm32f10x.h"
#include "stdbool.h"
#include "flash.h"

void flash_unlock(void){
    if (FLASH->CR & FLASH_CR_LOCK){
        FLASH->KEYR = 0x45670123u; FLASH->KEYR = 0xCDEF89ABu;
    }
}
void flash_lock(void){ FLASH->CR |= FLASH_CR_LOCK; }
void flash_wait(void){ while (FLASH->SR & FLASH_SR_BSY); }

void flash_erase_page(uint32_t addr){
    flash_wait(); FLASH->CR |= FLASH_CR_PER; FLASH->AR = addr; FLASH->CR |= FLASH_CR_STRT; flash_wait(); FLASH->CR &= ~FLASH_CR_PER;
}
void flash_prog_half(uint32_t addr, uint16_t v){
    flash_wait(); FLASH->CR |= FLASH_CR_PG; *(volatile uint16_t*)addr = v; flash_wait(); FLASH->CR &= ~FLASH_CR_PG;
}
//Boot is just jump to the application and run it
void jump_to_app(void){
	RCC_DeInit();
	SCB->SHCSR &= ~(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);
	__set_MSP(*(__IO uint32_t*)(APP_START_ADDR));
	uint32_t JumpAddress = *(__IO uint32_t*)(APP_START_ADDR + 4);
	void (*reset_handler)(void) = (void *)JumpAddress;
	reset_handler();	

}
static void flash_program(uint32_t dst_addr, const uint8_t* src, uint32_t len) {
FLASH_Unlock();
uint32_t i = 0;
while (i < len) {
uint16_t half;
if (i + 1 < len) half = (uint16_t)(src[i] | (src[i+1] << 8));
else half = (uint16_t)(src[i] | 0xFF00U); // pad last odd byte


while (FLASH_GetFlagStatus(FLASH_FLAG_BSY) == SET) {}
FLASH_ProgramHalfWord(dst_addr, half);
dst_addr += 2U; i += 2U;
}
FLASH_Lock();
}

static void flash_erase_app(uint32_t size_bytes) {
uint32_t pages = (size_bytes + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;
FLASH_Unlock();
for (uint32_t i = 0; i < pages; i++) {
uint32_t page_addr = APP_START_ADDR + i * FLASH_PAGE_SIZE;
while (FLASH_GetFlagStatus(FLASH_FLAG_BSY) == SET) {}
FLASH_ErasePage(page_addr);
}
FLASH_Lock();
}



static bool flash_erase_range(uint32_t start, uint32_t end) {
    // Erase page-by-page covering [start,end)
    uint32_t addr_aligned = start - (start % FLASH_PAGE_SIZE);
    for (uint32_t a = addr_aligned; a < end; a += FLASH_PAGE_SIZE) {
        // Skip pages that are fully before APP_START if you want; here we assume start>=APP_START
        while (FLASH_GetStatus() == FLASH_BUSY) {;}
        if (FLASH_ErasePage(a) != FLASH_COMPLETE) return false;
    }
    return true;
}
static bool flash_program_aligned(uint32_t addr, const uint8_t *data, uint32_t len) {
    // F1 can program half-words (16-bit) aligned to 2 bytes
    uint32_t i = 0;

    // Head: align to half-word boundary if needed
    if ((addr & 1U) != 0) return false; // enforce alignment

    while (i + 1 < len) {
        uint16_t hw = (uint16_t)(data[i] | (data[i+1] << 8));
        while (FLASH_GetStatus() == FLASH_BUSY) {;}
        if (FLASH_ProgramHalfWord(addr, hw) != FLASH_COMPLETE) return false;
        addr += 2;
        i += 2;
    }

    // Tail: if odd byte remains, read-modify-write the last halfword
    if (i < len) {
        uint16_t existing = *(volatile uint16_t *)(addr);
        uint16_t hw = (uint16_t)((existing & 0xFF00) | data[i]); // keep upper byte
        while (FLASH_GetStatus() == FLASH_BUSY) {;}
        if (FLASH_ProgramHalfWord(addr, hw) != FLASH_COMPLETE) return false;
    }
    return true;
}