#ifndef KIMHAL_H
#define KIMHAL_H

#include <Arduino.h>
#include <pgmspace.h> // Standard Arduino include
#include "MOS6502.h"
#include "TM1638.h"
#include "KIM_ROM.h"

// --- ESP8266 PINS ---
#define PIN_MODE_SELECT 13  //D7 on board, GPIO13 
#define PIN_TAPE_OUT    12 //D6 on board, GPIO12
#define PIN_TAPE_IN     A0

// [MEMORY CONFIG]
#define RAM_SIZE 8192 

class KIMHal {
  public:
    TM1638 hardware;
    uint8_t ram[RAM_SIZE]; 
    uint8_t riot[256];
    uint8_t port_b_out = 0xFF; 
    volatile uint32_t curkey = 0; 
    bool keypadMode = true; 

    // RIOT Timer State
    unsigned long timer_start_us = 0;
    uint8_t  timer_initial = 0;
    uint16_t timer_divider = 1;    
    bool     timer_irq_en = false;

    KIMHal() {
        memset(ram, 0, RAM_SIZE);
        memset(riot, 0, 256);
    }

    void begin() {
        hardware.begin();
        hardware.setupDisplay(true, 2);
        pinMode(PIN_TAPE_OUT, OUTPUT);
        digitalWrite(PIN_TAPE_OUT, LOW);
    }

    // --- TAPE HELPERS ---
    void sendPulse(uint8_t val) {
        int us = (val == 1) ? 138 : 208;
        digitalWrite(PIN_TAPE_OUT, HIGH);
        delayMicroseconds(us);
        digitalWrite(PIN_TAPE_OUT, LOW);
        delayMicroseconds(us);
    }
    void sendBit(uint8_t val) {
        int pulses = (val == 1) ? 9 : 6;
        for (int i = 0; i < pulses; i++) { sendPulse(val); }
    }
    void sendByte(uint8_t b) {
        for (int i = 0; i < 8; i++) { sendBit((b >> i) & 1); }
    }

    // --- KEY MATRIX (Copy exact same logic from previous KIMHal.h) ---
    void getKeyMatrix(uint32_t k, int &row, uint8_t &colMask) {
        row = -1; colMask = 0xFF;
        if (k == 0x4000040) { row = -1; colMask = 0xFF; return; } 
        if (k == 0x40000040) { row = -1; colMask = 0xFF; return; } 
        if (k == 0x44) { row = 2; colMask = ~0x02; return; } 
        if (k == 0x440) { row = 2; colMask = ~0x01; return; } 
        if (k == 0x4040) { row = 2; colMask = ~0x04; return; } 
        if (k == 0x4)        { row = 0; colMask = ~0x40; } 
        if (k == 0x400)      { row = 0; colMask = ~0x20; } 
        if (k == 0x4000)     { row = 0; colMask = ~0x10; } 
        if (k == 0x40000)    { row = 0; colMask = ~0x08; } 
        if (k == 0x400000)   { row = 0; colMask = ~0x04; } 
        if (k == 0x4000000)  { row = 0; colMask = ~0x02; } 
        if (k == 0x40000000) { row = 0; colMask = ~0x01; } 
        if (k == 0x2)        { row = 1; colMask = ~0x40; } 
        if (k == 0x20)       { row = 1; colMask = ~0x20; } 
        if (k == 0x200)      { row = 1; colMask = ~0x10; } 
        if (k == 0x2000)     { row = 1; colMask = ~0x08; } 
        if (k == 0x20000)    { row = 1; colMask = ~0x04; } 
        if (k == 0x200000)   { row = 1; colMask = ~0x02; } 
        if (k == 0x2000000)  { row = 1; colMask = ~0x01; } 
        if (k == 0x20000000) { row = 2; colMask = ~0x40; } 
        if (k == 0x1)        { row = 2; colMask = ~0x20; } 
        if (k == 0x10)       { row = 2; colMask = ~0x10; } 
        if (k == 0x100)      { row = 2; colMask = ~0x08; } 
    }

    // --- RIOT TIMER (Same logic) ---
    uint8_t getTimer(bool returnStatus) {
        unsigned long now = micros();
        unsigned long elapsed = now - timer_start_us;
        unsigned long ticks = elapsed / timer_divider;
        if (ticks <= timer_initial) {
            return timer_initial - (uint8_t)ticks;
        } else {
            unsigned long duration_us = (unsigned long)timer_initial * timer_divider;
            unsigned long overrun_us = elapsed - duration_us;
            uint8_t val = 0xFF - (uint8_t)(overrun_us & 0xFF);
            if (returnStatus) return val | 0x80; 
            return val;
        }
    }

    // --- BUS READ ---
    uint8_t read(uint16_t addr) {
        if (addr < 0x1000) return ram[addr];
        if (addr >= 0x2000 && addr <= 0x2FFF) return ram[addr - 0x1000]; 
        
        // Flash Read on ESP8266 (32-bit aligned access is preferred but byte read works)
        if (addr >= 0x1800 && addr <= 0x1FFF) return pgm_read_byte(&ROM_BIN[addr - 0x1800]);
        
        if (addr >= 0xFFFA) {
             if (addr == 0xFFFA) return ram[0x17FA];
             if (addr == 0xFFFB) return ram[0x17FB];
             if (addr == 0xFFFC) return pgm_read_byte(&ROM_BIN[0x7FC]); 
             if (addr == 0xFFFD) return pgm_read_byte(&ROM_BIN[0x7FD]);
             if (addr == 0xFFFE) return ram[0x17FE]; 
             if (addr == 0xFFFF) return ram[0x17FF];
        }

        if (addr >= 0x1700 && addr <= 0x17FF) {
            if (addr == 0x1706) return getTimer(false); 
            if (addr == 0x1707) return getTimer(true);  
            if (addr == 0x1740) {
                if (!keypadMode) return 0xFF; 
                int activeRow = (port_b_out >> 1) & 0x03; 
                int keyRow = -1;
                uint8_t keyColMask = 0xFF;
                if (curkey != 0) getKeyMatrix(curkey, keyRow, keyColMask);
                if (keyRow != -1 && keyRow == activeRow) return keyColMask; 
                return 0xFF; 
            }
            return riot[addr - 0x1700];
        }
        return 0x00;
    }

    // --- BUS WRITE ---
    void write(uint16_t addr, uint8_t val) {
        if (addr < 0x1000) { ram[addr] = val; return; }
        if (addr >= 0x2000 && addr <= 0x2FFF) { ram[addr - 0x1000] = val; return; }
        if (addr == 0xC000) { Serial.print(">> BENCH: "); Serial.println(millis()); return; }
        
        if (addr >= 0x1700 && addr <= 0x17FF) { 
            if ((addr >= 0x1704 && addr <= 0x1707) || (addr >= 0x170C && addr <= 0x170F)) {
                timer_initial = val;
                timer_start_us = micros();
                uint8_t mode = addr & 0x03; 
                if (mode == 0) timer_divider = 1;
                if (mode == 1) timer_divider = 8;
                if (mode == 2) timer_divider = 64;
                if (mode == 3) timer_divider = 1024;
                timer_irq_en = (addr >= 0x170C);
                return;
            }
            if (addr == 0x1742) port_b_out = val; 
            riot[addr - 0x1700] = val; 
            return; 
        }
    }
    
    // --- TRAP HANDLER ---
    void checkTraps(Mos6502* cpu) {
        if (cpu->pc < 0x1800) return;

        // Mode Select Trap
        if (cpu->pc == 0x1C54) { if (!keypadMode) { cpu->pc = 0x1C64; return; } }
        
        // SCANDS
        if (cpu->pc == 0x1F19) { 
            static unsigned long lastDisp = 0;
            // Display update overhead
            if (millis() - lastDisp > 20) {
                lastDisp = millis();
                hardware.writeDigit(0, ram[0xFB] >> 4); hardware.writeDigit(1, ram[0xFB] & 0x0F);
                hardware.writeDigit(2, ram[0xFA] >> 4); hardware.writeDigit(3, ram[0xFA] & 0x0F);
                hardware.writeDigit(4, ram[0xF9] >> 4); hardware.writeDigit(5, ram[0xF9] & 0x0F);
                hardware.setLed(0, (millis() / 500) % 2); 
            }
        }
        
        // TTY Input
        if (cpu->pc == 0x1E5A || cpu->pc == 0x1E61) {
            // Need to yield to ESP WiFi stack during wait loops
            while (!Serial.available()) { 
                yield(); // [CRITICAL] Keep ESP Watchdog happy
                hardware.writeDigit(0, ram[0xFB] >> 4); hardware.writeDigit(1, ram[0xFB] & 0x0F);
                hardware.writeDigit(2, ram[0xFA] >> 4); hardware.writeDigit(3, ram[0xFA] & 0x0F);
                hardware.writeDigit(4, ram[0xF9] >> 4); hardware.writeDigit(5, ram[0xF9] & 0x0F);
            }
            char k = Serial.read(); if (k >= 'a' && k <= 'z') k -= 32; 
            cpu->A = k; Serial.write(k); 
            uint8_t lo = cpu->Read(0x0100 + ++cpu->sp); uint8_t hi = cpu->Read(0x0100 + ++cpu->sp);
            cpu->pc = ((uint16_t)hi << 8) | lo; cpu->pc++;
            return; 
        }
        
        // TTY Output
        if (cpu->pc == 0x1EA0) {
            Serial.write(cpu->A);
            uint8_t lo = cpu->Read(0x0100 + ++cpu->sp); uint8_t hi = cpu->Read(0x0100 + ++cpu->sp);
            cpu->pc = ((uint16_t)hi << 8) | lo; cpu->pc++;
            return;
        }

        if (cpu->pc == 0x1C2F) { cpu->pc = 0x1C4F; return; }
    }
};
#endif