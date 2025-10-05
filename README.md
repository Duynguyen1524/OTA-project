# OTA-project

Firmware **Over-The-Air (OTA)** update for **STM32F1** (e.g., F103C8 “Blue Pill”) using an **ESP32** as the wireless bridge.  
The ESP32 receives a firmware image over Wi‑Fi and streams it to the STM32 bootloader/runtime over UART; the STM32 writes it to flash and swaps to the new image safely.

---

## ✨ Features

- OTA update of STM32 application via Wi‑Fi (ESP32 as transport)
- Chunked transfer with CRC verification per block and final image hash
- Flash erase/write with wear-safe page handling
- Minimal RTOS-style kernel (cooperative) for tasks/timers (`osKernel.*`, `Os.s`)
- Lightweight UART driver (`uart.*`) with ring buffer (I will optimize this later by changing it to SPI, I2C)
- Simple command protocol (`flash.*` handles parse + program)
- Rollback on verification failure (keeps previous image intact)

---

## 🔧 Hardware

- **Target MCU**: STM32F103Cx (tested), other STM32F1 likely similar
- **Transport**: ESP32-WROOM/DevKitC
- **Power**: 3.3 V for both boards (do **not** feed 5 V to STM32 IO)

### Wiring (UART1 example)

| STM32F1 | ESP32 | Notes                |
|---------|------:|----------------------|
| PA9  (TX)  | RX (GPIO3/U0RXD) | Cross TX→RX |
| PA10 (RX)  | TX (GPIO1/U0TXD) | Cross RX→TX |
| GND        | GND               | Common ground |
| 3V3        | 3V3               | Shared 3.3 V supply |

---

## 🧱 Software requirements

- **Toolchain**: `arm-none-eabi-gcc` (≥ 10) or STM32CubeIDE/Keil
- **Python 3** (optional) for a host-side sender script
- **ESP-IDF** or Arduino-ESP32 (for the ESP32 forwarder)

---

## 🔁 OTA flow (high level)

```
[Host PC] --Wi‑Fi TCP--> [ESP32] --UART--> [STM32F1]
                                 
```

- STM32 accumulates chunks, CRC-checks each, writes to flash pages.
- On `END`, STM32 recomputes whole-image hash and responds `OK/ERR`.
- On `COMMIT`, the boot record is updated and the app jumps to the new image.

---



## 🧰 Build & flash (STM32)

### With STM32CubeIDE
1. Create an STM32F103 project (empty).
2. Add `Os.s`, `osKernel.*`, `uart.*`, `flash.*` to `Core/Src` & `Core/Inc`.
3. Enable **USART1** with RX interrupt.
4. Build & debug as usual.

### With GCC/Make (example commands)
```bash
# Example only—adapt to your linker script and startup files
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -O2 -ffreestanding   -I. -c osKernel.c uart.c flash.c

arm-none-eabi-as -mcpu=cortex-m3 -mthumb Os.s -o Os.o

arm-none-eabi-gcc -Tstm32f103.ld -nostartfiles -Wl,--gc-sections   Os.o osKernel.o uart.o flash.o startup_stm32f103.o -o app.elf

arm-none-eabi-objcopy -O ihex app.elf app.hex
arm-none-eabi-objcopy -O binary app.elf app.bin
```

Flash with OpenOCD or ST-Link:
```bash
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c 'program app.hex verify reset exit'
```

---

## 📡 ESP32 bridge
Underconstruction 

---

## ▶️ Running an OTA update

1. Power both boards; ensure ESP32 ↔ STM32 UART is connected.
2. Start the ESP32 bridge (note its IP).
3. From your PC, run a sender script to push `app.bin`:

```bash
python tools/host/send_ota.py --ip 192.168.4.1 --port 8080 app.bin
```

**What you should see**
- Progress % on the host.
- STM32 printing states over UART (if `DEBUG_UART` enabled).
- Final `OK` then auto-reboot into the new app.

---
