# KIM-1 Emulator on ESP8266 (D1 Mini)

This port adapts the KIM-1 Emulator to run on the ESP8266 (specifically the Wemos/Lolin D1 Mini). It leverages the higher clock speed of the ESP8266 (running at 160 MHz) to achieve close to realistic ~1 MHz emulation speed while driving the TM1638 display.

## Hardware Requirements

* **Microcontroller:** Wemos D1 Mini (ESP8266) or equivalent NodeMCU.
* **Display:** TM1638 LED & Key module (or DsKy for Raspberry Pi).
* **Logic Shifter:** 4-Channel Bi-Directional Logic Level Converter (3.3V <-> 5V). **Required** for reliable key reading.
* **Wires:** Jumpers for breadboarding.

## Wiring Guide (Conflict-Free Pinout)

Because the ESP8266 has specific boot requirements on certain pins (D3, D4, D8), this project uses a specific pinout to ensure the board boots correctly even when connected to the display.

**Note:** You **must** use a Logic Level Shifter between the ESP8266 and the TM1638. The TM1638 requires 5V for proper display brightness, but the ESP8266 inputs are 3.3V and cannot reliably read the 5V return signals without shifting.

| Signal Function | ESP8266 Pin (LV Side) | GPIO | DsKy / TM1638 Pin (HV Side) |
| :--- | :--- | :--- | :--- |
| **DIO (Data)** | **D1** | GPIO 5 | Pin 35 (DIO) |
| **STB (Strobe)** | **D0** | GPIO 16 | Pin 37 (STB1) |
| **CLK (Clock)** | **D5** | GPIO 14 | Pin 33 (CLK) |
| **Mode Jumper** | **D7** | GPIO 13 | *Connect to GND for TTY Mode* |
| **Tape Out** | **D6** | GPIO 12 | Audio Out (Future) |
| **Tape In** | **A0** | ADC0 | Audio In (Future) |

* **LV (Low Voltage):** Connect to ESP8266 3.3V.
* **HV (High Voltage):** Connect to USB 5V (Vin).
* **GND:** Connect Grounds from both sides together.

## Installation & Configuration

### Arduino IDE Settings
To ensure the emulator runs at full speed and uploads correctly on "Clone" boards:

1.  **Board:** `LOLIN(WEMOS) D1 R2 & mini`
2.  **CPU Frequency:** `160 MHz` (Critical for speed)
3.  **Upload Speed:** `115200` (Recommended for stability on clones)
4.  **IwIP Variant:** `v2 Lower Memory`

### Boot Mode "Jumper"
* **Keypad Mode (Default):** Leave Pin **D7** unconnected (Internal Pull-up). The system boots into standard KIM-1 mode using the LED display.
* **TTY Mode:** Connect Pin **D7** to **GND** before booting. The system redirects input/output to the Serial Monitor (115200 baud) for running Tiny Basic or the Monitor.

### Performance
* **Target Speed:** 1.000 MHz (Standard KIM-1)
* **Emulated Speed:** ~0.96 - 1.00 MHz
* **Instruction Support:** Full 6502 instruction set (MOS6502 core).

## Troubleshooting "Clone" Boards
If you encounter "Connecting..." errors during upload:
1.  **Disconnect** the TM1638 module (power drain can affect USB).
2.  **Force Flash Mode:** Connect **D3 (GPIO 0)** to **GND**, press Reset, then start the upload. Remove the jumper once uploading starts.
