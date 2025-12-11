#ifndef MOS6502_H
#define MOS6502_H

#include <stdint.h>
#include <Arduino.h> 

// --- GLOBAL STATUS FLAGS ---
#define F_C 0x01 // Carry
#define F_Z 0x02 // Zero
#define F_I 0x04 // Interrupt Disable
#define F_D 0x08 // Decimal
#define F_B 0x10 // Break
#define F_K 0x20 // Constant (Unused)
#define F_V 0x40 // Overflow
#define F_N 0x80 // Negative

// Compatibility aliases
#define CONSTANT F_K
#define BREAK    F_B

// Forward Declaration
class Mos6502;

class Mos6502 {
public:
    // Registers (Public for HAL/Debug)
    uint8_t A, X, Y, sp;
    uint16_t pc;
    uint8_t status;
    
    // Debug Flag
    bool debug = false;
    
    // [REMOVED] SystemBus* bus; 

    // Constructor (No pointer needed anymore)
    Mos6502();

    void Reset();
    void Run(int32_t cycles);
    void NMI();

    // Memory Accessors
    uint8_t Read(uint16_t addr);
    void Write(uint16_t addr, uint8_t val);

    // Helpers
    void StackPush(uint8_t byte);
    uint8_t StackPop();
    bool getFlag(uint8_t f) { return (status & f); }

private:
    typedef void (Mos6502::*CodeExec)(uint16_t);
    typedef uint16_t (Mos6502::*AddrExec)();

    struct Instr {
        AddrExec addr;
        CodeExec code;
        uint8_t cycles;
    };

    static Instr InstrTable[256];
    void Exec(Instr i);

    // Opcodes
    void Op_ADC(uint16_t src); void Op_AND(uint16_t src); void Op_ASL(uint16_t src); 
    void Op_ASL_ACC(uint16_t src); void Op_BCC(uint16_t src); void Op_BCS(uint16_t src);
    void Op_BEQ(uint16_t src); void Op_BIT(uint16_t src); void Op_BMI(uint16_t src); 
    void Op_BNE(uint16_t src); void Op_BPL(uint16_t src); void Op_BRK(uint16_t src);
    void Op_BVC(uint16_t src); void Op_BVS(uint16_t src); void Op_CLC(uint16_t src); 
    void Op_CLD(uint16_t src); void Op_CLI(uint16_t src); void Op_CLV(uint16_t src);
    void Op_CMP(uint16_t src); void Op_CPX(uint16_t src); void Op_CPY(uint16_t src);
    void Op_DEC(uint16_t src); void Op_DEX(uint16_t src); void Op_DEY(uint16_t src);
    void Op_EOR(uint16_t src); void Op_INC(uint16_t src); void Op_INX(uint16_t src); 
    void Op_INY(uint16_t src); void Op_JMP(uint16_t src); void Op_JSR(uint16_t src);
    void Op_LDA(uint16_t src); void Op_LDX(uint16_t src); void Op_LDY(uint16_t src);
    void Op_LSR(uint16_t src); void Op_LSR_ACC(uint16_t src); void Op_NOP(uint16_t src); 
    void Op_ORA(uint16_t src); void Op_PHA(uint16_t src); void Op_PHP(uint16_t src);
    void Op_PLA(uint16_t src); void Op_PLP(uint16_t src); void Op_ROL(uint16_t src); 
    void Op_ROL_ACC(uint16_t src); void Op_ROR(uint16_t src); void Op_ROR_ACC(uint16_t src);
    void Op_RTI(uint16_t src); void Op_RTS(uint16_t src); void Op_SBC(uint16_t src); 
    void Op_SEC(uint16_t src); void Op_SED(uint16_t src); void Op_SEI(uint16_t src); 
    void Op_STA(uint16_t src); void Op_STX(uint16_t src); void Op_STY(uint16_t src);
    void Op_TAX(uint16_t src); void Op_TAY(uint16_t src); void Op_TSX(uint16_t src); 
    void Op_TXA(uint16_t src); void Op_TXS(uint16_t src); void Op_TYA(uint16_t src);
    void Op_ILLEGAL(uint16_t src);

    uint16_t Addr_ACC(); uint16_t Addr_IMM(); uint16_t Addr_ABS();
    uint16_t Addr_ZER(); uint16_t Addr_ZEX(); uint16_t Addr_ZEY();
    uint16_t Addr_ABX(); uint16_t Addr_ABY(); uint16_t Addr_IMP();
    uint16_t Addr_REL(); uint16_t Addr_INX(); uint16_t Addr_INY();
    uint16_t Addr_ABI();
};

#endif