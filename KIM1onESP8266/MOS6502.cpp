#include "MOS6502.h"
#include "KIMHal.h" // [NEW] Direct Include for performance

// [NEW] Link to the global 'hal' object defined in the .ino file
extern KIMHal hal; 

// Helper Macros
#define SET_NEGATIVE(x)  ((x) ? (status |= F_N) : (status &= ~F_N))
#define SET_OVERFLOW(x)  ((x) ? (status |= F_V) : (status &= ~F_V))
#define SET_CONSTANT(x)  ((x) ? (status |= F_K) : (status &= ~F_K))
#define SET_BREAK(x)     ((x) ? (status |= F_B) : (status &= ~F_B))
#define SET_DECIMAL(x)   ((x) ? (status |= F_D) : (status &= ~F_D))
#define SET_INTERRUPT(x) ((x) ? (status |= F_I) : (status &= ~F_I))
#define SET_ZERO(x)      ((x) ? (status |= F_Z) : (status &= ~F_Z))
#define SET_CARRY(x)     ((x) ? (status |= F_C) : (status &= ~F_C))

#define IF_NEGATIVE()  ((status & F_N) ? true : false)
#define IF_OVERFLOW()  ((status & F_V) ? true : false)
#define IF_CONSTANT()  ((status & F_K) ? true : false)
#define IF_BREAK()     ((status & F_B) ? true : false)
#define IF_DECIMAL()   ((status & F_D) ? true : false)
#define IF_INTERRUPT() ((status & F_I) ? true : false)
#define IF_ZERO()      ((status & F_Z) ? true : false)
#define IF_CARRY()     ((status & F_C) ? true : false)

// --- LINKING ---
// [CHANGED] Direct calls to 'hal' remove virtual function overhead
uint8_t Mos6502::Read(uint16_t addr) { return hal.read(addr); }
void Mos6502::Write(uint16_t addr, uint8_t val) { hal.write(addr, val); }

Mos6502::Instr Mos6502::InstrTable[256];

// [CHANGED] Constructor no longer takes system pointer
Mos6502::Mos6502() {
   A = 0; X = 0; Y = 0;
   sp = 0xFD;
   status = 0x24; 
   debug = false;

   static bool initialized = false;
   if (initialized) return;
   initialized = true;

   Instr instr;
   instr.addr = &Mos6502::Addr_IMP;
   instr.code = &Mos6502::Op_ILLEGAL;
   instr.cycles = 0;
   for(int i = 0; i < 256; i++) InstrTable[i] = instr;

   #define MAKE(HEX, CODE, MODE, CYCLES) \
      instr.code = &Mos6502::Op_ ## CODE; \
      instr.addr = &Mos6502::Addr_ ## MODE; \
      instr.cycles = CYCLES; \
      InstrTable[HEX] = instr;

   // Opcodes
   MAKE(0x69, ADC, IMM, 2); MAKE(0x65, ADC, ZER, 3); MAKE(0x75, ADC, ZEX, 4);
   MAKE(0x6D, ADC, ABS, 4); MAKE(0x7D, ADC, ABX, 4); MAKE(0x79, ADC, ABY, 4);
   MAKE(0x61, ADC, INX, 6); MAKE(0x71, ADC, INY, 5);

   MAKE(0x29, AND, IMM, 2); MAKE(0x25, AND, ZER, 3); MAKE(0x35, AND, ZEX, 4);
   MAKE(0x2D, AND, ABS, 4); MAKE(0x3D, AND, ABX, 4); MAKE(0x39, AND, ABY, 4);
   MAKE(0x21, AND, INX, 6); MAKE(0x31, AND, INY, 5);

   MAKE(0x0A, ASL_ACC, ACC, 2); MAKE(0x06, ASL, ZER, 5); MAKE(0x16, ASL, ZEX, 6);
   MAKE(0x0E, ASL, ABS, 6); MAKE(0x1E, ASL, ABX, 7);

   MAKE(0x90, BCC, REL, 2); MAKE(0xB0, BCS, REL, 2); MAKE(0xF0, BEQ, REL, 2);
   MAKE(0x30, BMI, REL, 2); MAKE(0xD0, BNE, REL, 2); MAKE(0x10, BPL, REL, 2);
   MAKE(0x50, BVC, REL, 2); MAKE(0x70, BVS, REL, 2);

   MAKE(0x24, BIT, ZER, 3); MAKE(0x2C, BIT, ABS, 4);
   MAKE(0x00, BRK, IMP, 7);

   MAKE(0x18, CLC, IMP, 2); MAKE(0xD8, CLD, IMP, 2); MAKE(0x58, CLI, IMP, 2);
   MAKE(0xB8, CLV, IMP, 2); MAKE(0x38, SEC, IMP, 2); MAKE(0xF8, SED, IMP, 2);
   MAKE(0x78, SEI, IMP, 2);

   MAKE(0xC9, CMP, IMM, 2); MAKE(0xC5, CMP, ZER, 3); MAKE(0xD5, CMP, ZEX, 4);
   MAKE(0xCD, CMP, ABS, 4); MAKE(0xDD, CMP, ABX, 4); MAKE(0xD9, CMP, ABY, 4);
   MAKE(0xC1, CMP, INX, 6); MAKE(0xD1, CMP, INY, 5);

   MAKE(0xE0, CPX, IMM, 2); MAKE(0xE4, CPX, ZER, 3); MAKE(0xEC, CPX, ABS, 4);
   MAKE(0xC0, CPY, IMM, 2); MAKE(0xC4, CPY, ZER, 3); MAKE(0xCC, CPY, ABS, 4);

   MAKE(0xC6, DEC, ZER, 5); MAKE(0xD6, DEC, ZEX, 6); MAKE(0xCE, DEC, ABS, 6);
   MAKE(0xDE, DEC, ABX, 7);
   MAKE(0xCA, DEX, IMP, 2); MAKE(0x88, DEY, IMP, 2);

   MAKE(0x49, EOR, IMM, 2); MAKE(0x45, EOR, ZER, 3); MAKE(0x55, EOR, ZEX, 4);
   MAKE(0x4D, EOR, ABS, 4); MAKE(0x5D, EOR, ABX, 4); MAKE(0x59, EOR, ABY, 4);
   MAKE(0x41, EOR, INX, 6); MAKE(0x51, EOR, INY, 5);

   MAKE(0xE6, INC, ZER, 5); MAKE(0xF6, INC, ZEX, 6); MAKE(0xEE, INC, ABS, 6);
   MAKE(0xFE, INC, ABX, 7);
   MAKE(0xE8, INX, IMP, 2); MAKE(0xC8, INY, IMP, 2);

   MAKE(0x4C, JMP, ABS, 3); MAKE(0x6C, JMP, ABI, 5);
   MAKE(0x20, JSR, ABS, 6);

   MAKE(0xA9, LDA, IMM, 2); MAKE(0xA5, LDA, ZER, 3); MAKE(0xB5, LDA, ZEX, 4);
   MAKE(0xAD, LDA, ABS, 4); MAKE(0xBD, LDA, ABX, 4); MAKE(0xB9, LDA, ABY, 4);
   MAKE(0xA1, LDA, INX, 6); MAKE(0xB1, LDA, INY, 5);

   MAKE(0xA2, LDX, IMM, 2); MAKE(0xA6, LDX, ZER, 3); MAKE(0xB6, LDX, ZEY, 4);
   MAKE(0xAE, LDX, ABS, 4); MAKE(0xBE, LDX, ABY, 4);

   MAKE(0xA0, LDY, IMM, 2); MAKE(0xA4, LDY, ZER, 3); MAKE(0xB4, LDY, ZEX, 4);
   MAKE(0xAC, LDY, ABS, 4); MAKE(0xBC, LDY, ABX, 4);

   MAKE(0x4A, LSR_ACC, ACC, 2); MAKE(0x46, LSR, ZER, 5); MAKE(0x56, LSR, ZEX, 6);
   MAKE(0x4E, LSR, ABS, 6); MAKE(0x5E, LSR, ABX, 7);

   MAKE(0xEA, NOP, IMP, 2);

   MAKE(0x09, ORA, IMM, 2); MAKE(0x05, ORA, ZER, 3); MAKE(0x15, ORA, ZEX, 4);
   MAKE(0x0D, ORA, ABS, 4); MAKE(0x1D, ORA, ABX, 4); MAKE(0x19, ORA, ABY, 4);
   MAKE(0x01, ORA, INX, 6); MAKE(0x11, ORA, INY, 5);

   MAKE(0x48, PHA, IMP, 3); MAKE(0x08, PHP, IMP, 3);
   MAKE(0x68, PLA, IMP, 4); MAKE(0x28, PLP, IMP, 4);

   MAKE(0x2A, ROL_ACC, ACC, 2); MAKE(0x26, ROL, ZER, 5); MAKE(0x36, ROL, ZEX, 6);
   MAKE(0x2E, ROL, ABS, 6); MAKE(0x3E, ROL, ABX, 7);

   MAKE(0x6A, ROR_ACC, ACC, 2); MAKE(0x66, ROR, ZER, 5); MAKE(0x76, ROR, ZEX, 6);
   MAKE(0x6E, ROR, ABS, 6); MAKE(0x7E, ROR, ABX, 7);

   MAKE(0x40, RTI, IMP, 6); MAKE(0x60, RTS, IMP, 6);

   MAKE(0xE9, SBC, IMM, 2); MAKE(0xE5, SBC, ZER, 3); MAKE(0xF5, SBC, ZEX, 4);
   MAKE(0xED, SBC, ABS, 4); MAKE(0xFD, SBC, ABX, 4); MAKE(0xF9, SBC, ABY, 4);
   MAKE(0xE1, SBC, INX, 6); MAKE(0xF1, SBC, INY, 5);

   MAKE(0x85, STA, ZER, 3); MAKE(0x95, STA, ZEX, 4); MAKE(0x8D, STA, ABS, 4);
   MAKE(0x9D, STA, ABX, 5); MAKE(0x99, STA, ABY, 5); MAKE(0x81, STA, INX, 6);
   MAKE(0x91, STA, INY, 6);

   MAKE(0x86, STX, ZER, 3); MAKE(0x96, STX, ZEY, 4); MAKE(0x8E, STX, ABS, 4);
   MAKE(0x84, STY, ZER, 3); MAKE(0x94, STY, ZEX, 4); MAKE(0x8C, STY, ABS, 4);

   MAKE(0xAA, TAX, IMP, 2); MAKE(0xA8, TAY, IMP, 2); MAKE(0xBA, TSX, IMP, 2);
   MAKE(0x8A, TXA, IMP, 2); MAKE(0x9A, TXS, IMP, 2); MAKE(0x98, TYA, IMP, 2);
}

void Mos6502::Run(int32_t cycles) {
   uint8_t opcode;
   Instr instr;
   while(cycles > 0) {
      // [CHANGED] Direct call to 'hal'
      hal.checkTraps(this); 
      
      // DEBUG TRACER
      if (debug) {
          char buf[64];
          // [CHANGED] Direct 'hal' read
          sprintf(buf, "PC:%04X Op:%02X A:%02X X:%02X Y:%02X S:%02X P:%02X", 
                  pc, hal.read(pc), A, X, Y, sp, status);
          Serial.println(buf);
      }

      opcode = Read(pc++);
      instr = InstrTable[opcode];
      Exec(instr);
      cycles -= instr.cycles;
   }
}

void Mos6502::Reset() {
   sp = 0xFF;
   status = 0x24;
   pc = (Read(0xFFFD) << 8) | Read(0xFFFC);
}

void Mos6502::NMI() {
    // 1. Push PC High Byte
    // [CHANGED] Direct writes
    hal.write(0x0100 + sp, (pc >> 8) & 0xFF);
    sp--; 

    // 2. Push PC Low Byte
    hal.write(0x0100 + sp, pc & 0xFF);
    sp--;

    // 3. Push Status Register
    hal.write(0x0100 + sp, status | 0x20); 
    sp--;

    // 4. Disable Interrupts
    status |= F_I; 

    // 5. Fetch Vector from FFFA/FFFB
    uint8_t lo = Read(0xFFFA);
    uint8_t hi = Read(0xFFFB);
    
    pc = (hi << 8) | lo;
}

// --- ADDRESSING MODES ---
uint16_t Mos6502::Addr_ACC() { return 0; }
uint16_t Mos6502::Addr_IMM() { return pc++; }
uint16_t Mos6502::Addr_ABS() { uint16_t addr = Read(pc++); addr |= (Read(pc++) << 8); return addr; }
uint16_t Mos6502::Addr_ZER() { return Read(pc++); }
uint16_t Mos6502::Addr_IMP() { return 0; }
uint16_t Mos6502::Addr_REL() {
   uint16_t offset = (uint16_t)Read(pc++);
   if (offset & 0x80) offset |= 0xFF00;
   return pc + (int16_t)offset;
}
uint16_t Mos6502::Addr_ABI() {
   uint16_t addrL = Read(pc++);
   uint16_t addrH = Read(pc++);
   uint16_t abs = (addrH << 8) | addrL;
   uint16_t effL = Read(abs);
   uint16_t effH = Read((abs & 0xFF00) + ((abs + 1) & 0x00FF));
   return effL + 0x100 * effH;
}
uint16_t Mos6502::Addr_ZEX() { return (Read(pc++) + X) & 0xFF; }
uint16_t Mos6502::Addr_ZEY() { return (Read(pc++) + Y) & 0xFF; }
uint16_t Mos6502::Addr_ABX() { uint16_t addr = Read(pc++); addr |= (Read(pc++) << 8); return addr + X; }
uint16_t Mos6502::Addr_ABY() { uint16_t addr = Read(pc++); addr |= (Read(pc++) << 8); return addr + Y; }
uint16_t Mos6502::Addr_INX() {
   uint16_t zeroL = (Read(pc++) + X) & 0xFF;
   return Read(zeroL) + (Read((zeroL + 1) & 0xFF) << 8);
}
uint16_t Mos6502::Addr_INY() {
   uint16_t zeroL = Read(pc++);
   uint16_t addr = Read(zeroL) + (Read((zeroL + 1) & 0xFF) << 8);
   return addr + Y;
}

// --- STACK ---
void Mos6502::StackPush(uint8_t byte) {
   Write(0x0100 + sp, byte);
   if(sp == 0x00) sp = 0xFF; else sp--;
}
uint8_t Mos6502::StackPop() {
   if(sp == 0xFF) sp = 0x00; else sp++;
   return Read(0x0100 + sp);
}

void Mos6502::Exec(Instr i) {
   uint16_t src = (this->*i.addr)();
   (this->*i.code)(src);
}

void Mos6502::Op_ILLEGAL(uint16_t src) {}

// --- OPCODES ---
void Mos6502::Op_ADC(uint16_t src) {
   uint8_t m = Read(src);
   unsigned int tmp = m + A + (IF_CARRY() ? 1 : 0);
   SET_OVERFLOW(!((A ^ m) & 0x80) && ((A ^ tmp) & 0x80));
   SET_NEGATIVE(tmp & 0x80);
   SET_ZERO(!(tmp & 0xFF));
   if (IF_DECIMAL()) {
      int AL = ((A & 0xF) + (m & 0xF) + (IF_CARRY() ? 1 : 0));
      if (AL >= 0xA) AL = ((AL + 6) & 0xF) + 0x10;
      tmp = (m & 0xF0) + (A & 0xF0) + AL;
      SET_NEGATIVE(tmp & 0x80);
      if (tmp >= 0xA0) tmp += 0x60;
   }
   SET_CARRY(tmp > 0xFF);
   A = tmp & 0xFF;
}

void Mos6502::Op_AND(uint16_t src) {
   uint8_t m = Read(src);
   uint8_t res = m & A;
   SET_NEGATIVE(res & 0x80); SET_ZERO(!res); A = res;
}

void Mos6502::Op_ASL(uint16_t src) {
   uint8_t m = Read(src);
   SET_CARRY(m & 0x80); m <<= 1;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_ASL_ACC(uint16_t src) {
   uint8_t m = A;
   SET_CARRY(m & 0x80); m <<= 1;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}

void Mos6502::Op_BCC(uint16_t src) { if (!IF_CARRY()) pc = src; }
void Mos6502::Op_BCS(uint16_t src) { if (IF_CARRY()) pc = src; }
void Mos6502::Op_BEQ(uint16_t src) { if (IF_ZERO()) pc = src; }
void Mos6502::Op_BMI(uint16_t src) { if (IF_NEGATIVE()) pc = src; }
void Mos6502::Op_BNE(uint16_t src) { if (!IF_ZERO()) pc = src; }
void Mos6502::Op_BPL(uint16_t src) { if (!IF_NEGATIVE()) pc = src; }
void Mos6502::Op_BVC(uint16_t src) { if (!IF_OVERFLOW()) pc = src; }
void Mos6502::Op_BVS(uint16_t src) { if (IF_OVERFLOW()) pc = src; }

void Mos6502::Op_BIT(uint16_t src) {
   uint8_t m = Read(src);
   uint8_t res = m & A;
   status = (status & 0x3F) | (uint8_t)(m & 0xC0);
   SET_ZERO(!res);
}

void Mos6502::Op_BRK(uint16_t src) {
   pc++;
   StackPush((pc >> 8) & 0xFF); StackPush(pc & 0xFF);
   StackPush(status | CONSTANT | BREAK);
   SET_INTERRUPT(1);
   pc = (Read(0xFFFF) << 8) + Read(0xFFFE);
}

void Mos6502::Op_CLC(uint16_t src) { SET_CARRY(0); }
void Mos6502::Op_CLD(uint16_t src) { SET_DECIMAL(0); }
void Mos6502::Op_CLI(uint16_t src) { SET_INTERRUPT(0); }
void Mos6502::Op_CLV(uint16_t src) { SET_OVERFLOW(0); }

void Mos6502::Op_CMP(uint16_t src) {
   unsigned int tmp = A - Read(src);
   SET_CARRY(tmp < 0x100);
   SET_NEGATIVE(tmp & 0x80);
   SET_ZERO(!(tmp & 0xFF));
}
void Mos6502::Op_CPX(uint16_t src) {
   unsigned int tmp = X - Read(src);
   SET_CARRY(tmp < 0x100);
   SET_NEGATIVE(tmp & 0x80);
   SET_ZERO(!(tmp & 0xFF));
}
void Mos6502::Op_CPY(uint16_t src) {
   unsigned int tmp = Y - Read(src);
   SET_CARRY(tmp < 0x100);
   SET_NEGATIVE(tmp & 0x80);
   SET_ZERO(!(tmp & 0xFF));
}

void Mos6502::Op_DEC(uint16_t src) {
   uint8_t m = Read(src); m--;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_DEX(uint16_t src) { X--; SET_NEGATIVE(X & 0x80); SET_ZERO(!X); }
void Mos6502::Op_DEY(uint16_t src) { Y--; SET_NEGATIVE(Y & 0x80); SET_ZERO(!Y); }

void Mos6502::Op_EOR(uint16_t src) {
   uint8_t m = Read(src); m = A ^ m;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}

void Mos6502::Op_INC(uint16_t src) {
   uint8_t m = Read(src); m++;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_INX(uint16_t src) { X++; SET_NEGATIVE(X & 0x80); SET_ZERO(!X); }
void Mos6502::Op_INY(uint16_t src) { Y++; SET_NEGATIVE(Y & 0x80); SET_ZERO(!Y); }

void Mos6502::Op_JMP(uint16_t src) { pc = src; }
void Mos6502::Op_JSR(uint16_t src) {
   pc--;
   StackPush((pc >> 8) & 0xFF); StackPush(pc & 0xFF);
   pc = src;
}

void Mos6502::Op_LDA(uint16_t src) {
   uint8_t m = Read(src);
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}
void Mos6502::Op_LDX(uint16_t src) {
   uint8_t m = Read(src);
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); X = m;
}
void Mos6502::Op_LDY(uint16_t src) {
   uint8_t m = Read(src);
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Y = m;
}

void Mos6502::Op_LSR(uint16_t src) {
   uint8_t m = Read(src);
   SET_CARRY(m & 0x01); m >>= 1;
   SET_NEGATIVE(0); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_LSR_ACC(uint16_t src) {
   uint8_t m = A;
   SET_CARRY(m & 0x01); m >>= 1;
   SET_NEGATIVE(0); SET_ZERO(!m); A = m;
}

void Mos6502::Op_NOP(uint16_t src) {}
void Mos6502::Op_ORA(uint16_t src) {
   uint8_t m = Read(src); m = A | m;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}

void Mos6502::Op_PHA(uint16_t src) { StackPush(A); }
void Mos6502::Op_PHP(uint16_t src) { StackPush(status | CONSTANT | BREAK); }
void Mos6502::Op_PLA(uint16_t src) { A = StackPop(); SET_NEGATIVE(A & 0x80); SET_ZERO(!A); }
void Mos6502::Op_PLP(uint16_t src) { status = (status & (CONSTANT | BREAK)) | (StackPop() & ~(CONSTANT | BREAK)); }

void Mos6502::Op_ROL(uint16_t src) {
   uint16_t m = Read(src); m <<= 1;
   if (IF_CARRY()) m |= 0x01;
   SET_CARRY(m > 0xFF); m &= 0xFF;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_ROL_ACC(uint16_t src) {
   uint16_t m = A; m <<= 1;
   if (IF_CARRY()) m |= 0x01;
   SET_CARRY(m > 0xFF); m &= 0xFF;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}

void Mos6502::Op_ROR(uint16_t src) {
   uint16_t m = Read(src);
   if (IF_CARRY()) m |= 0x100;
   SET_CARRY(m & 0x01); m >>= 1; m &= 0xFF;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); Write(src, m);
}
void Mos6502::Op_ROR_ACC(uint16_t src) {
   uint16_t m = A;
   if (IF_CARRY()) m |= 0x100;
   SET_CARRY(m & 0x01); m >>= 1; m &= 0xFF;
   SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m;
}

void Mos6502::Op_RTI(uint16_t src) {
   uint8_t lo, hi;
   status = (status & (CONSTANT | BREAK)) | (StackPop() & ~(CONSTANT | BREAK));
   lo = StackPop(); hi = StackPop();
   pc = (hi << 8) | lo;
}

void Mos6502::Op_RTS(uint16_t src) {
   uint8_t lo = StackPop(); uint8_t hi = StackPop();
   pc = ((hi << 8) | lo) + 1;
}

void Mos6502::Op_SBC(uint16_t src) {
   uint8_t m = Read(src);
   int tmp = A - m - (IF_CARRY() ? 0 : 1);
   SET_OVERFLOW(((A ^ m) & (A ^ tmp) & 0x80) != 0);
   SET_NEGATIVE(tmp & 0x80 );
   SET_ZERO(!(tmp & 0xFF));
   if (IF_DECIMAL()) {
      int AL = (A & 0x0F) - (m & 0x0F) - (IF_CARRY() ? 0 : 1);
      if (AL < 0) AL = ((AL - 6) & 0x0F) - 0x10;
      tmp = (A & 0xF0) - (m & 0xF0) + AL;
      SET_NEGATIVE(tmp & 0x80 );
      if (tmp < 0) tmp -= 0x60;
   }
   SET_CARRY(tmp >= 0);
   A = tmp & 0xFF;
}

void Mos6502::Op_SEC(uint16_t src) { SET_CARRY(1); }
void Mos6502::Op_SED(uint16_t src) { SET_DECIMAL(1); }
void Mos6502::Op_SEI(uint16_t src) { SET_INTERRUPT(1); }

void Mos6502::Op_STA(uint16_t src) { Write(src, A); }
void Mos6502::Op_STX(uint16_t src) { Write(src, X); }
void Mos6502::Op_STY(uint16_t src) { Write(src, Y); }

void Mos6502::Op_TAX(uint16_t src) { A = A; SET_NEGATIVE(A & 0x80); SET_ZERO(!A); X = A; }
void Mos6502::Op_TAY(uint16_t src) { A = A; SET_NEGATIVE(A & 0x80); SET_ZERO(!A); Y = A; }
void Mos6502::Op_TSX(uint16_t src) { uint8_t m = sp; SET_NEGATIVE(m & 0x80); SET_ZERO(!m); X = m; }
void Mos6502::Op_TXA(uint16_t src) { uint8_t m = X; SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m; }
void Mos6502::Op_TXS(uint16_t src) { sp = X; }
void Mos6502::Op_TYA(uint16_t src) { uint8_t m = Y; SET_NEGATIVE(m & 0x80); SET_ZERO(!m); A = m; }