#ifndef FLASH_H
#define FLASH_H
#include <stdint.h>
#include "stdbool.h"
#define APP_START_ADDR 0x08008000UL
#define FLASH_PAGE_SIZE 1024U
#define FLASH_END_ADDR          0x08010000UL
void flash_unlock(void);
void flash_lock(void);
void flash_wait(void);
void flash_erase_page(uint32_t addr);
void flash_prog_half(uint32_t addr, uint16_t v);
void jump_to_app(void);
static void flash_program(uint32_t dst_addr, const uint8_t* src, uint32_t len);
static void flash_erase_app(uint32_t size_bytes);
static bool flash_erase_range(uint32_t start, uint32_t end);
static bool flash_program_aligned(uint32_t addr, const uint8_t *data, uint32_t len);
#endif
