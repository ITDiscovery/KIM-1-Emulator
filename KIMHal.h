#ifndef KIMHAL_H
#define KIMHAL_H

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "MOS6502.h"
#include "TM1638.h"
#include "KIM_ROM.h"

#define PIN_MODE_SELECT PIN_PD7 
#define PIN_TAPE_OUT    PIN_PD6

// [MEMORY CONFIG]
// 8KB Total RAM (Split into two 4KB banks)
// Bank 0: 0x0000 - 0x0FFF (System Low RAM)
// Bank 1: 0x2000 - 0x2FFF (Expansion/BASIC RAM)
#define RAM_SIZE 8192 

// [CHANGED] No longer inherits from SystemBus
class KIMHal {
  public:
    TM1638 hardware;
    
    uint8_t ram[RAM_SIZE]; 
    uint8_t riot[256];
    
    uint8_t port_b_out = 0xFF; 
    volatile uint32_t curkey = 0; 
    
    bool keypadMode = true; 

    // --- RIOT TIMER STATE ---
    unsigned long timer_start_us = 0;
    uint8_t  timer_initial = 0;
    uint16_t timer_divider = 1;     // 1, 8, 64, 1024
    bool     timer_irq_en = false;  // Is interrupt enabled?

    KIMHal() {
        memset(ram, 0, RAM_SIZE);
        memset(riot, 0, 256);
    }

    void begin() {
        hardware.begin();
        hardware.setupDisplay(true, 2);
        // Setup for Tape out Pin
        pinMode(PIN_TAPE_OUT, OUTPUT);
        digitalWrite(PIN_TAPE_OUT, LOW);
    }

    // --- TAPE OUTPUT HELPERS ---
    void sendPulse(uint8_t val) {
        int us = (val == 1) ? 138 : 208;
        digitalWrite(PIN_TAPE_OUT, HIGH);
        delayMicroseconds(us);
        digitalWrite(PIN_TAPE_OUT, LOW);
        delayMicroseconds(us);
    }
    void sendBit(uint8_t val) {
        int pulses = (val == 1) ? 9 : 6;
        for (int i = 0; i < pulses; i++) {
            sendPulse(val);
        }
    }
    void sendByte(uint8_t b) {
        for (int i = 0; i < 8; i++) {
            sendBit((b >> i) & 1);
        }
    }

    // --- KEY MATRIX DEFINITION ---
    void getKeyMatrix(uint32_t k, int &row, uint8_t &colMask) {
        row = -1; colMask = 0xFF;

        // --- RESERVED AUX FUNCTIONS ---
        if (k == 0x4000040) { row = -1; colMask = 0xFF; return; } // Aux1 (Shift+5)
        if (k == 0x40000040) { row = -1; colMask = 0xFF; return; } // Aux2 (Shift+6)

        // --- SHIFT FUNCTIONS ---
        if (k == 0x44) { row = 2; colMask = ~0x02; return; } // GO
        if (k == 0x440) { row = 2; colMask = ~0x01; return; } // PC
        if (k == 0x4040) { row = 2; colMask = ~0x04; return; } // +

        // --- SINGLE KEYS ---
        // ROW 0
        if (k == 0x4)        { row = 0; colMask = ~0x40; } // 0
        if (k == 0x400)      { row = 0; colMask = ~0x20; } // 1
        if (k == 0x4000)     { row = 0; colMask = ~0x10; } // 2
        if (k == 0x40000)    { row = 0; colMask = ~0x08; } // 3
        if (k == 0x400000)   { row = 0; colMask = ~0x04; } // 4
        if (k == 0x4000000)  { row = 0; colMask = ~0x02; } // 5
        if (k == 0x40000000) { row = 0; colMask = ~0x01; } // 6

        // ROW 1
        if (k == 0x2)        { row = 1; colMask = ~0x40; } // 7
        if (k == 0x20)       { row = 1; colMask = ~0x20; } // 8
        if (k == 0x200)      { row = 1; colMask = ~0x10; } // 9
        if (k == 0x2000)     { row = 1; colMask = ~0x08; } // A
        if (k == 0x20000)    { row = 1; colMask = ~0x04; } // B
        if (k == 0x200000)   { row = 1; colMask = ~0x02; } // C
        if (k == 0x2000000)  { row = 1; colMask = ~0x01; } // D

        // ROW 2
        if (k == 0x20000000) { row = 2; colMask = ~0x40; } // E
        if (k == 0x1)        { row = 2; colMask = ~0x20; } // F
        if (k == 0x10)       { row = 2; colMask = ~0x10; } // AD
        if (k == 0x100)      { row = 2; colMask = ~0x08; } // DA
    }

    // --- RIOT TIMER ENGINE ---
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
    // [CHANGED] 'override' removed
    uint8_t read(uint16_t addr) {
        // [BANK 0] Low RAM (0000 - 0FFF)
        if (addr < 0x1000) {
            return ram[addr];
        }

        // [BANK 1] High RAM (2000 - 2FFF)
        if (addr >= 0x2000 && addr <= 0x2FFF) {
            return ram[addr - 0x1000]; 
        }
        
        // ROM (1800-1FFF)
        if (addr >= 0x1800 && addr <= 0x1FFF) return pgm_read_byte(&ROM_BIN[addr - 0x1800]);
        
        // Vectors (FFFA-FFFF)
        if (addr >= 0xFFFA) {
             if (addr == 0xFFFA) return ram[0x17FA]; // NMI (RAM)
             if (addr == 0xFFFB) return ram[0x17FB];
             if (addr == 0xFFFC) return pgm_read_byte(&ROM_BIN[0x7FC]); // RST (ROM)
             if (addr == 0xFFFD) return pgm_read_byte(&ROM_BIN[0x7FD]);
             if (addr == 0xFFFE) return ram[0x17FE]; // IRQ (RAM)
             if (addr == 0xFFFF) return ram[0x17FF];
        }

        // I/O (1700-17FF)
        if (addr >= 0x1700 && addr <= 0x17FF) {
            // RIOT TIMER READ
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
    // [CHANGED] 'override' removed
    void write(uint16_t addr, uint8_t val) {
        // [BANK 0] Low RAM (0000 - 0FFF)
        if (addr < 0x1000) { 
            ram[addr] = val; 
            return; 
        }

        // [BANK 1] High RAM (2000 - 2FFF)
        if (addr >= 0x2000 && addr <= 0x2FFF) {
            ram[addr - 0x1000] = val; 
            return;
        }

        // BENCHMARK TRIGGER (Address $C000)
        if (addr == 0xC000) {
            Serial.print(">> BENCHMARK TRIGGER: ");
            Serial.println(millis());
            return;
        }
        
        // I/O
        if (addr >= 0x1700 && addr <= 0x17FF) { 
            // RIOT TIMER WRITE
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
    // [CHANGED] 'override' removed
    void checkTraps(Mos6502* cpu) {
        // 1. FAST EXIT: If PC is in User RAM (< 1800), return immediately
        if (cpu->pc < 0x1800) return;

        // Mode Select Trap
        if (cpu->pc == 0x1C54) { if (!keypadMode) { cpu->pc = 0x1C64; return; } }
        
        // SCANDS
        if (cpu->pc == 0x1F19) { 
            static unsigned long lastDisp = 0;
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
            while (!Serial.available()) { 
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

        // DET1 Bypass
        if (cpu->pc == 0x1C2F) { cpu->pc = 0x1C4F; return; }

        // TAPE DUMP TRAP (Address $1800)
        if (cpu->pc == 0x1800) {
            Serial.println(">> TAPE DUMP STARTED...");
            uint16_t startAddr = ram[0x17F5] | (ram[0x17F6] << 8);
            uint16_t endAddr   = ram[0x17F7] | (ram[0x17F8] << 8);
            uint8_t  tapeID    = ram[0x17F9];
            Serial.print("   ID: "); Serial.print(tapeID, HEX);
            Serial.print(" Start: "); Serial.print(startAddr, HEX);
            Serial.print(" End: "); Serial.println(endAddr, HEX);

            for (int i=0; i<100; i++) sendByte(0x16);
            sendByte(0x2A);
            sendByte(tapeID);
            sendByte(startAddr & 0xFF);
            sendByte(startAddr >> 8);
            
            uint16_t current = startAddr;
            while (current <= endAddr) {
                sendByte(read(current));
                current++;
            }
            sendByte(0x00); sendByte(0x00); // Checksum
            sendByte(0x04); sendByte(0x04); // EOT
            
            Serial.println(">> TAPE DUMP COMPLETE.");
            uint8_t lo = cpu->Read(0x0100 + ++cpu->sp); 
            uint8_t hi = cpu->Read(0x0100 + ++cpu->sp);
            cpu->pc = ((uint16_t)hi << 8) | lo; 
            cpu->pc++;
            return;
        }
    }
};

#endif