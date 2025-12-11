#include <ESP8266WiFi.h> // Include to disable WiFi
#include "KIMHal.h"
#include "MOS6502.h"
#include "TinyBasic.h"

KIMHal hal;
Mos6502 cpu; 

// --- HELPER: DUMP REGISTERS ---
void dumpRegisters() {
  Serial.println("\n--- CPU STATE ---");
  Serial.print("PC: "); Serial.print(cpu.pc, HEX);
  Serial.print(" A:  "); Serial.print(cpu.A, HEX);
  Serial.print(" X:  "); Serial.print(cpu.X, HEX);
  Serial.print(" Y:  "); Serial.print(cpu.Y, HEX);
  Serial.print(" SP: "); Serial.print(cpu.sp, HEX);
  Serial.print(" SR: "); Serial.println(cpu.status, HEX);
  Serial.println("-----------------");
}

// --- HELPER: BENCHMARK LOADER (Shift+6) ---
// Kept this for diagnostic purposes
void loadAndRunBenchmark() {
    Serial.println("\n--- LOADING BENCHMARK ---");
    const uint8_t bench[] = {
        0x8D, 0x00, 0xC0, 0xA2, 0xFF, 0xA0, 0xFF, 0x88, 0xD0, 0xFD, 
        0xCA, 0xD0, 0xF8, 0xC6, 0x00, 0xD0, 0xF2, 0x8D, 0x00, 0xC0, 
        0x4C, 0x14, 0x02 
    };
    for(int i=0; i<sizeof(bench); i++) hal.write(0x0200 + i, bench[i]);
    hal.write(0x0000, 0x05); 
    Serial.println("Running 1,633,319 cycles (Target: ~1633ms)...");
    cpu.pc = 0x0200;
}

// --- HELPER: SYSTEM KEYS ---
void handleSystemKeys(uint32_t raw) {
  // ST (Stop/NMI) = Shift + 3
  if (raw == 0x40040) {
      cpu.NMI();
      Serial.println(">> NMI (STOP) TRIGGERED");
      dumpRegisters();
      while(hal.hardware.readButtons() == 0x40040) delay(10);
  }

  // RS (Reset) = Shift + 4
  if (raw == 0x400040) {
      cpu.Reset();
      if (hal.keypadMode) cpu.pc = 0x1C4F; 
      Serial.println(">> SYSTEM RESET");
      while(hal.hardware.readButtons() == 0x400040) delay(10);
  }

  // BENCHMARK (Aux2) = Shift + 6
  if (raw == 0x40000040) {
      if (cpu.debug) { cpu.debug = false; Serial.println(">> Debug auto-disabled"); }
      loadAndRunBenchmark();
      while(hal.hardware.readButtons() == 0x40000040) delay(10);
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  // Wait a moment for Serial to stabilize, but don't hang if no USB
  delay(100); 
  
  Serial.println("\n--- KIM-1 on ESP8266 Ready ---");
  
  hal.begin(); 
  
  pinMode(PIN_MODE_SELECT, INPUT_PULLUP);
  delay(10);
  
  if (digitalRead(PIN_MODE_SELECT) == LOW) {
      hal.keypadMode = false;
      Serial.println(">> Jumper Detected: TTY Mode Active");
      // --- LOAD TINY BASIC INTO RAM ($2000) ---
      // This simulates an expansion card always being present.
      Serial.print("Loading Tiny Basic ("); 
      Serial.print(sizeof(TINY_BASIC_BIN)); 
      Serial.println(" bytes) to $2000...");
  
      for (uint16_t i = 0; i < sizeof(TINY_BASIC_BIN); i++) {
        uint8_t b = pgm_read_byte(&TINY_BASIC_BIN[i]);
        hal.write(0x2000 + i, b);
      }
      Serial.println("Done. To start: Type [2000] [GO].");
  } else {
      hal.keypadMode = true;
      Serial.println(">> No Jumper: Keypad Mode Active");
  }

  // Init Display RAM to default "0000 00"
  hal.ram[0xF9] = 0x00; 
  hal.ram[0xFA] = 0x00; 
  hal.ram[0xFB] = 0x00;

  // Init Vectors (In RIOT RAM 17xx)
  hal.write(0x17FA, 0x00); hal.write(0x17FB, 0x1C); // NMI -> 1C00
  hal.write(0x17FE, 0x00); hal.write(0x17FF, 0x1C); // IRQ -> 1C00

  cpu.Reset(); 
  
  // If Hardwired TTY mode, jump straight to monitor entry
  if (!hal.keypadMode) {
      cpu.pc = 0x1C00;
  }
}

// --- TEMPORARY DIAGNOSTIC LOOP ---
void loop() {
  // 1. Run CPU briefly (keep the display alive)
  cpu.Run(500); 
  yield(); 

  // 2. Read the Keypad
  uint32_t raw = hal.hardware.readButtons();
  hal.curkey = raw;

  #ifdef DEBUG
  // 3. SPY ON THE RAW DATA
  static uint32_t lastRaw = 0;
  if (raw != lastRaw && raw != 0) {
      Serial.print(">> KEY RAW: 0x");
      Serial.println(raw, HEX);
      lastRaw = raw;
  }
  if (raw == 0) lastRaw = 0; // Reset on release
  #endif

  // 4. Handle Reset (Shift+RS) just in case you need to reboot
  if (raw == 0x400040) { // Assuming standard mapping for now
      cpu.Reset();
      Serial.println(">> RESET");
      delay(500);
  }
}