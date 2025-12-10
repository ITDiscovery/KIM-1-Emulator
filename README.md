# KIM-1 Emulator on AVR128DB28
**Phase 1 Completion Report**

## 1. Executive Summary
This project implements a cycle-accurate(ish) MOS 6502 emulator on the AVR128DB28 microcontroller. It replicates the functionality of the original MOS KIM-1 (1976), including the 6-digit LED display, 23-key hex keypad, TTY (Teletype) serial interface, and cassette tape output.

**Key Achievement:** The system successfully runs Tom Pittman's Tiny BASIC (v2.1) directly from an emulated ROM, interacting via USB Serial just as the original hardware would have over a current loop interface.

## 2. Hardware Specification
* **Host MCU:** AVR128DB28 (Running at 24 MHz internal clock)
* **User Interface:** TM1638 LED & Keypad Module (8 LEDs, 8 Buttons, 8x 7-Segment Displays)
* **Communication:** USB Serial (115200 Baud) simulating the TTY interface.
* **Resource Usage:**
    * **Flash:** 21,886 bytes (16%)
    * **SRAM:** 11,589 bytes (70%) - Primarily used by the 8KB allocated for 6502 RAM.

### Pinout Configuration
| AVR Pin | Function | Description |
| :--- | :--- | :--- |
| **PA4** | `PIN_STB` | TM1638 Strobe |
| **PA5** | `PIN_DIO` | TM1638 Data I/O |
| **PA6** | `PIN_CLK` | TM1638 Clock |
| **PD6** | `PIN_TAPE_OUT` | Audio/Tape Data Output |
| **PD7** | `PIN_MODE_SELECT` | Jumper: **LOW** = TTY Mode, **HIGH** (Open) = Keypad Mode |

---

## 3. Architecture & Software Design

### The "Devirtualized" Core
To maximize performance on the 8-bit AVR architecture, standard C++ object-oriented features (virtual functions) were stripped out in the final optimization pass. The `Mos6502` class communicates directly with the global `KIMHal` instance, eliminating pointer indirection overhead.

### Memory Map (Emulated 6502 Bus)
The emulator provides 8KB of RAM split into two banks to mirror a fully expanded KIM-1 system.

| Address Range | Description | Host Implementation |
| :--- | :--- | :--- |
| **$0000 - $03FF** | Zero Page & Stack | Mapped to `ram[0..1023]` |
| **$0400 - $0FFF** | System RAM (User Code) | Mapped to `ram[1024..4095]` |
| **$1700 - $17FF** | RIOT I/O & Timer | Emulated IO / `riot[]` array |
| **$1800 - $1FFF** | KIM-1 Monitor ROM | Read from `ROM_BIN` (PROGMEM) |
| **$2000 - $2FFF** | Expansion RAM | Mapped to `ram[4096..8191]` |
| **$FFFA - $FFFF** | Vectors (NMI/RST/IRQ) | Mapped to RAM vectors or ROM |

### The Time-Slice Scheduler
Unlike standard Arduino sketches that run one CPU instruction per loop, this system uses a **Time-Slice Scheduler**.
1.  **Burst Mode:** The CPU executes instructions in 10ms bursts (approx. 2,200 cycles).
2.  **UI Overhead:** The slow TM1638 display update and key scanning (~1ms) only occur *between* bursts.
3.  **Result:** This prevents the display logic from slowing down the CPU, maintaining smooth execution.

---

## 4. Performance

* **Real KIM-1 Speed:** 1.000 MHz
* **Emulator Speed:** ~0.222 MHz (222 kHz)
* **Relative Speed:** ~22% of original hardware.

While slower than the original, 222 kHz is significantly faster than standard Arduino Uno-based emulators (typically ~100 kHz) and is fully sufficient for the KIM-1 Monitor, Tiny BASIC, and educational usage.

---

## 5. User Manual

### Mode Selection
The system mode is determined at startup by the jumper on **PD7**:
* **Keypad Mode (No Jumper):** System boots into the LED scanning routine. TTY traps are active but the display is primary.
* **TTY Mode (Jumper Grounded):** System forces execution to `$1C00`, bypassing the LED display for pure Serial interaction.

### Keypad Mapping (TM1638)
The emulator maps the physical 8 buttons of the TM1638 to the 23 keys of the KIM-1 using a **Shift** modifier.

| Function | Physical Keys |
| :--- | :--- |
| **0 - F** | Direct mapping or Shift combinations (See `TM1638.h`) |
| **AD** (Address Mode) | Key `A` (Unshifted) |
| **DA** (Data Mode) | Key `D` (Unshifted) |
| **+** (Increment) | Shift + `1` |
| **PC** (Program Counter) | Shift + `0` |
| **GO** (Execute) | Shift + `2` |
| **ST** (Stop / NMI) | Shift + `3` |
| **RS** (Reset) | Shift + `4` |

### Running Tiny BASIC
Tom Pittman's Tiny BASIC is stored in Flash memory and automatically loaded into RAM bank `$2000` on startup.

1.  Connect via Serial Monitor (115200 Baud).
2.  On the keypad, press: `2` `0` `0` `0` (Select Address).
3.  Press: `GO` (Shift + 2).
4.  Switch to Serial Terminal. You may need to press `ENTER` to see the prompt.
5.  Prompt is `>`.

**Example Session:**
```basic
> 10 PRINT "HELLO WORLD"
> 20 GOTO 10
> RUN
HELLO WORLD
HELLO WORLD
```

**Project Files**

- KIM1onAVR128.ino: Main entry point, scheduler, and System Key handler.
- KIMHal.h: Hardware Abstraction Layer. Handles Memory decoding, TTY Traps, and RIOT Timer emulation.
- MOS6502.cpp/h: The CPU Core. Optimized for AVR.
- TM1638.h: Driver for the specific LED/Keypad module.
- KIM_ROM.h: Hex dump of the original KIM-1 Monitor (002 and 003 ROMs).
- TinyBasic.h: Hex dump of Tom Pittman's Tiny BASIC (v2.1). 

7. Future Features and Projects
Although Phase 1 is complete, future enhancements could include:
- EEPROM Storage: Saving BASIC programs to the AVR's internal EEPROM to persist them after power loss.
- Tape Input: Using the AVR's Analog Comparator to read actual KIM-1 cassette tapes.
- Re-align to ESP8266 for speed increase.
- Add 6809, 8086 and Z80 emulations
