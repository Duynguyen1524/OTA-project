# OTA-project

Firmware **Over-The-Air (OTA)** update for **STM32F1** (e.g., F103C8 “Blue Pill”) using an **ESP32** as the wireless bridge.  
The ESP32 receives a firmware image over Wi‑Fi and streams it to the STM32 bootloader/runtime over UART; the STM32 writes it to flash and swaps to the new image safely.

---

## ✨ Features

- OTA update of STM32 application via Wi‑Fi (ESP32 as transport)
- Chunked transfer with CRC verification per block and final image hash
- Flash erase/write with wear-safe page handling
- Minimal RTOS-style kernel (cooperative) for tasks/timers (`osKernel.*`, `Os.s`)
- Lightweight UART driver (`uart.*`) with ring buffer
- Simple command protocol (`flash.*` handles parse + program)
- Rollback on verification failure (keeps previous image intact)

---

## 📂 Repository layout

```
.
├── Os.s             # Context switch / minimal scheduler primitives (ARM asm)
├── osKernel.c/.h    # Tiny cooperative "OS" (tasks, delays)
├── uart.c/.h        # UART init, IRQ, RX/TX ring buffers
├── flash.c/.h       # Flash page ops, CRC, OTA command handler
├── README.md        # This file
```
> If you add the ESP32 sketch or Python sender later, place them under `tools/esp32/` and `tools/host/` respectively.

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

Optional:
- Boot0 pin accessible (if you use ST ROM bootloader path)
- A status LED on PC13 for update progress

---

## 🧱 Software requirements

- **Toolchain**: `arm-none-eabi-gcc` (≥ 10) or STM32CubeIDE/Keil
- **Python 3** (optional) for a host-side sender script
- **ESP-IDF** or Arduino-ESP32 (for the ESP32 forwarder)

---

## 🔁 OTA flow (high level)

```
[Host PC] --Wi‑Fi TCP--> [ESP32] --UART--> [STM32F1]
                                  |
                                  | 1) HELLO → capabilities
                                  | 2) BEGIN  → image size, hash
                                  | 3) DATA   → N x {seq, payload, CRC}
                                  | 4) END    → final hash verify
                                  | 5) COMMIT → swap/jump
```

- STM32 accumulates chunks, CRC-checks each, writes to flash pages.
- On `END`, STM32 recomputes whole-image hash and responds `OK/ERR`.
- On `COMMIT`, the boot record is updated and the app jumps to the new image.

---

## 🧵 Protocol (default values)

- **Baud**: 115200 8-N-1  
- **Chunk size**: 512 bytes payload (tune with `FLASH_CHUNK_SZ`)
- **CRC**: CRC-32 (polynomial 0x04C11DB7) per chunk
- **Image hash**: CRC-32 or (optionally) SHA-256 if you enable it

Message frames (little-endian):
```
| SOF 0x55AA | CMD (1B) | SEQ (2B) | LEN (2B) | PAYLOAD (LEN) | CRC32 (4B) |
```

**Commands**
- `0x01 HELLO` → reply: version, page size
- `0x02 BEGIN` (total_size, image_crc)
- `0x03 DATA`  (offset, bytes…)
- `0x04 END`
- `0x05 COMMIT`
- `0x06 ABORT`

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

Two common options:

**Arduino-ESP32 sketch (quick)**
- Creates a Wi‑Fi AP or joins your network.
- Opens TCP port `:8080`; for each connection, forwards framed bytes to UART.
- Uses `Serial2.begin(115200, SERIAL_8N1, RX, TX);`

**ESP-IDF app (robust)**
- FreeRTOS tasks for socket RX/TX and UART RX/TX with back-pressure.
- Optional TLS and basic auth.

Put the source under `tools/esp32/` when you add it.

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

## 🔒 Safety & rollback

- Writes occur to a **secondary region**; the running app is not overwritten.
- `COMMIT` flips a small boot record in flash.
- If power fails mid-update, the old image remains valid; on next boot the
  bootloader verifies the committed slot and rolls back if needed.

> Adjust flash layout constants in `flash.h`:
> - `FLASH_PAGE_SIZE`, `FLASH_BASE`, `APP_SLOT_A`, `APP_SLOT_B`, etc.

---

## 🧪 Testing

- Unit-ish tests: build with `-DTEST_CRC` to self-check CRC routines
- Fuzz: feed random chunk order (sender can shuffle) → expect `ABORT`
- End-to-end: 100× small updates + 10× large (~64–128 KB) without error

---

## ⚙️ Configuration switches (`flash.h` / `uart.h`)

- `UART_BAUD` (default 115200)
- `FLASH_CHUNK_SZ` (default 512)
- `ENABLE_SHA256` (off by default)
- `DEBUG_UART` (prints state transitions)

---

## 🗺️ Roadmap

- [ ] Add ESP32 source to repo
- [ ] Optional TLS & signed images (Ed25519)
- [ ] Progress LED + watchdog integration
- [ ] CI: build on push, size/CRC artifact, basic unit tests

---

## ❓ FAQ

**Q:** Why ESP32 instead of direct Wi‑Fi on STM32?  
**A:** Simpler, cheaper, and keeps the STM32 firmware minimal; ESP32 handles networking.

**Q:** Can I use the ST ROM bootloader?  
**A:** Yes—adapt the sender to its protocol, but this project uses an **in‑app** updater for flexibility and rollback.

---

## 📜 License

MIT (recommended). Add a `LICENSE` file to confirm.

---

## 🙌 Acknowledgments

Thanks to the STM32 and ESP32 communities for reference drivers and bootloader notes.

---

### Quick start checklist

- [ ] Wire ESP32 ↔ STM32 (PA9/PA10 ↔ RX/TX, GND common)
- [ ] Build & flash STM32 app once via ST‑Link
- [ ] Load ESP32 bridge
- [ ] `send_ota.py app.bin` → watch progress → device reboots into new image
