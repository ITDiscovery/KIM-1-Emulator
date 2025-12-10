#include "KIMHal.h"
#include "MOS6502.h"
#include "TinyBasic.h" // [NEW] Include the ROM header

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
  // Wait a moment for Serial to stabilize, but don't hang if no USB
  delay(1000); 
  
  Serial.println("\n--- KIM-1 Emulator Ready ---");
  
  hal.begin(); 
  
  pinMode(PIN_MODE_SELECT, INPUT_PULLUP);
  delay(10);
  
  if (digitalRead(PIN_MODE_SELECT) == LOW) {
      hal.keypadMode = false;
      Serial.println(">> Jumper Detected: TTY Mode Active");
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

  cpu.Reset(); 
  
  // If Hardwired TTY mode, jump straight to monitor entry
  if (!hal.keypadMode) {
      cpu.pc = 0x1C00;
  }
}

// --- MAIN LOOP (SCHEDULER) ---
void loop() {
  // CONFIG: Run ~10ms of emulation before checking UI
  const unsigned long BATCH_DURATION_US = 10000; 
  unsigned long startSlice = micros();

  // 1. CPU BURST LOOP
  while (micros() - startSlice < BATCH_DURATION_US) {
      cpu.Run(500); 
      if (cpu.debug) break; 
  }

  // 2. UI UPDATE (Overhead)
  uint32_t raw = hal.hardware.readButtons();
  hal.curkey = raw;

  // 3. HEARTBEAT (LED 7)
  static int hrtbt = 0;
  if ((millis() % 500) < 250) {
      if(hrtbt == 0) { hal.hardware.setLed(7, 1); hrtbt = 1; }
  } else {
      if(hrtbt == 1) { hal.hardware.setLed(7, 0); hrtbt = 0; }
  }

  // 4. CHECK COMMAND KEYS
  handleSystemKeys(raw);
}