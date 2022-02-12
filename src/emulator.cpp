#include "emulator.hpp"

#include <iostream>
#include <unistd.h>
#include <climits>

#include "instruction.hpp"

Emulator::Emulator()
{}

int Emulator::run(const std::string& inFilename)
{
    int res = EE_OK;

    res = init(inFilename);
    if (res == EE_OK) {
        res = loop();
        terminate();
    }

    std::cout << std::endl;

    return res;
}

int Emulator::init(const std::string& inFilename)
{
    std::ifstream inFile(inFilename, std::ios_base::in);
    if (!inFile.is_open()) {
        std::cout << "Cannot open file for reading: " << inFilename << std::endl;
        return EE_FILE;
    }
    inFilename_ = inFilename;
    
    // init memory
    memory_.resize(MEMORY_SIZE);

    std::string line;
    ushort addr;
    while (std::getline(inFile, line)) {
        if (sscanf(line.c_str(), "%hx", &addr) == 0) {
            std::cout << "Invalid hex file format" << std::endl;
            return EE_FILE;
        }
        ubyte *memBlock = memory_.data() + addr;
        sscanf(line.c_str(),
            "%*x%*c%hhx%hhx%hhx%hhx%hhx%hhx%hhx%hhx",
            memBlock,
            memBlock+1,
            memBlock+2,
            memBlock+3,
            memBlock+4,
            memBlock+5,
            memBlock+6,
            memBlock+7
        );
    }

    // init registers
    registers_[REG_PC] = readWord(IRQ_ADDR(IRQ_RESET));
    registers_[REG_SP] = INITIAL_SP;
    registers_[REG_PSW] = INITIAL_PSW;

    // init memory mapped registers
    termIn_ = 0;
    termOut_ = 0;
    timCfg_ = INITIAL_TIM_CFG;

    // init irq
    irq_ = 0;

    // configure terminal
    termios tRawStdin;
    tcgetattr(STDIN_FILENO, &tOldStdin_);
    tRawStdin = tOldStdin_;
    tRawStdin.c_lflag &= ~(ICANON | ECHO);
    tRawStdin.c_cc[VMIN] = 0;
    tRawStdin.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &tRawStdin);
    // no output buffering
    setbuf(stdout, NULL);

    // init timer
    interval_ = TIM_CFG_INTERVALS[timCfg_];
    t0_ = std::chrono::high_resolution_clock::now();

    running_ = true;

    return EE_OK;
}

void Emulator::terminate()
{
    memory_.clear();

    // restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &tOldStdin_);
}

int Emulator::loop()
{
    while (running_) {
        handleInput();
        updateTimer();
        instr();
        intr();
    }

    return EE_OK;
}

void Emulator::handleInput()
{
    char in;
    if (read(STDIN_FILENO, &in, 1) == 0)
        return;

    // exit emulator
    if (in == 3 || in == 26) // ctrl+c | ctrl+z
        running_ = false;

    writeWord(MREG_TERM_IN, (ushort)in);
    interruptRequest(IRQ_TERMINAL);
}

void Emulator::updateTimer()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = (t1 - t0_) / std::chrono::milliseconds(1);
    if (elapsed >= interval_) {
        t0_ = std::chrono::high_resolution_clock::now();
        interruptRequest(IRQ_TIMER);
    }
}

void Emulator::instr()
{
    if (fetch() != EE_OK) {
        interruptRequest(IRQ_USAGE_FAULT);
        return;
    }
    if (decode() != EE_OK) {
        interruptRequest(IRQ_USAGE_FAULT);
        return;
    }
    if (execute() != EE_OK) {
        interruptRequest(IRQ_USAGE_FAULT);
        return;
    }
    writeback();
}

int Emulator::fetch()
{
    ushort& pc = registers_[REG_PC];
    ir_[IR_InstrDescr] = memory_[pc++]; // InstrDescr

    switch (ir_[IR_InstrDescr]) {
    case OC_HALT:
    case OC_IRET:
    case OC_RET:
        return EE_OK; // instr size: 1 byte
    case OC_INT:
    case OC_CALL:
    case OC_JMP:
    case OC_JEQ:
    case OC_JNE:
    case OC_JGT:
    case OC_XCHG:
    case OC_ADD:
    case OC_SUB:
    case OC_MUL:
    case OC_DIV:
    case OC_CMP:
    case OC_NOT:
    case OC_AND:
    case OC_OR:
    case OC_XOR:
    case OC_TEST:
    case OC_SHL:
    case OC_SHR:
    case OC_LDR:
    case OC_STR:
        break;
    default:
        return EE_OPCODE; // invalid opcode
    }

    ir_[IR_RegsDescr] = memory_[pc++]; // RegsDescr

    switch (IR_InstrDescr_OC(ir_)) {
    case OC_INT:
    case OC_XCHG:
    case OC_GROUP_ARITHMETICAL:
    case OC_GROUP_LOGICAL:
    case OC_GROUP_SHIFT:
        return EE_OK; // instr size: 2 bytes
    }

    ir_[IR_AddrMode] = memory_[pc++]; // AddrMode

    switch (IR_AddrMode_AM(ir_)) {
    case AM_REGDIR:
    case AM_REGIND:
        return EE_OK; // instr size: 3 bytes
    case AM_IMMED:
        if (ir_[IR_InstrDescr] == OC_STR)
            return EE_ADDR_MODE;
    case AM_MEMDIR:
    case AM_REGDIR_OFFSET:
    case AM_REGIND_OFFSET:
        break;
    default:
        return EE_ADDR_MODE; // invalid addr mode
    }

    // instr size: 5 bytes

    ir_[IR_DataHigh] = memory_[pc++]; // DataHigh
    ir_[IR_DataLow] = memory_[pc++]; // DataLow

    return EE_OK;
}

int Emulator::decode()
{
    ubyte regD = IR_RegsDescr_RD(ir_);
    ubyte regS = IR_RegsDescr_RS(ir_);

    // check register validity
    switch (ir_[IR_InstrDescr]) {
    case OC_HALT:
    case OC_IRET:
    case OC_RET:
        break;
    case OC_XCHG:
    case OC_ADD:
    case OC_SUB:
    case OC_MUL:
    case OC_DIV:
    case OC_CMP:
    case OC_AND:
    case OC_OR:
    case OC_XOR:
    case OC_TEST:
    case OC_SHL:
    case OC_SHR:
        if (regS >= NUM_REGISTERS)
            return EE_REG;
        B_ = registers_[IR_RegsDescr_RS(ir_)];
    case OC_INT:
    case OC_NOT:
        if (regD >= NUM_REGISTERS)
            return EE_REG;
        A_ = registers_[IR_RegsDescr_RD(ir_)];
        break;
    case OC_LDR:
    case OC_STR:
        if (regD >= NUM_REGISTERS)
            return EE_REG;
        A_ = registers_[IR_RegsDescr_RD(ir_)];
    case OC_CALL:
    case OC_JMP:
    case OC_JEQ:
    case OC_JNE:
    case OC_JGT:
        switch (IR_AddrMode_AM(ir_)) {
        case AM_IMMED:
            B_ = IR_Data(ir_);
            break;
        case AM_REGDIR:
            if (regS >= NUM_REGISTERS)
                return EE_REG;
            if (ir_[IR_InstrDescr] != OC_STR)
                B_ = registers_[IR_RegsDescr_RS(ir_)];
            break;
        case AM_REGDIR_OFFSET:
            if (regS >= NUM_REGISTERS)
                return EE_REG;
            if (ir_[IR_InstrDescr] != OC_STR)
                B_ = registers_[IR_RegsDescr_RS(ir_)] + IR_Data(ir_);
            else
                B_ = IR_Data(ir_);
            break;
        case AM_REGIND: {
            if (regS >= NUM_REGISTERS)
                return EE_REG;
            ushort& regS = registers_[IR_RegsDescr_RS(ir_)];
            preUpdateRegister(regS);
            if (ir_[IR_InstrDescr] != OC_STR)
                B_ = readWord(regS);
            else
                B_ = regS;
            postUpdateRegister(regS);
        }
            break;
        case AM_REGIND_OFFSET: {
            if (regS >= NUM_REGISTERS)
                return EE_REG;
            ushort& regS = registers_[IR_RegsDescr_RS(ir_)];
            preUpdateRegister(regS);
            if (ir_[IR_InstrDescr] != OC_STR)
                B_ = readWord(regS + IR_Data(ir_));
            else
                B_ = regS + IR_Data(ir_);
            postUpdateRegister(regS);
        }
            break;
        case AM_MEMDIR:
            if (ir_[IR_InstrDescr] != OC_STR)
                B_ = readWord(IR_Data(ir_));
            else
                B_ = IR_Data(ir_);
            break;
        }
        break;
    }

    return EE_OK;
}

int Emulator::execute()
{
    ushort& pc = registers_[REG_PC];
    ushort& psw = registers_[REG_PSW];

    switch (ir_[IR_InstrDescr]) {
    case OC_HALT:
        running_ = false;
        break;
    case OC_IRET:
        pc = pop();
        psw = pop();
        break;
    case OC_RET:
        pc = pop();
        break;
    case OC_INT:
        if (A_ < IVT_SIZE)
            interruptRequest((IRQNo)A_);
        else
            return EE_OPERAND; // invalid ivt table entry
        break;
    case OC_CALL:
        push(pc);
        pc = B_;
        break;
    case OC_JMP:
        pc = B_;
        break;
    case OC_JEQ:
        if (psw & PSW_Z) // Z == 1
            pc = B_;
        break;
    case OC_JNE:
        if (!(psw & PSW_Z)) // Z == 0
            pc = B_;
        break;
    case OC_JGT:
        if (((psw & PSW_N) == (psw & PSW_O)) && !(psw & PSW_Z)) // (N == O) && (Z == 0)
            pc = B_;
        break;
    case OC_XCHG:
        break;
    case OC_ADD:
        C_ = A_ + B_;
        break;
    case OC_SUB:
        C_ = A_ - B_;
        break;
    case OC_MUL:
        C_ = A_ * B_;
        break;
    case OC_DIV:
        C_ = A_ / B_;
        break;
    case OC_CMP: {
        psw &= ~(PSW_Z | PSW_O | PSW_C | PSW_N);
        B_ = -B_;
        uint temp = A_ + B_;
        bool Asgn = A_ & USHORT_SIGN_BIT;
        bool Bsgn = B_ & USHORT_SIGN_BIT;
        bool tempsgn = temp & USHORT_SIGN_BIT;
        if ((ushort)temp == 0)
            psw |= PSW_Z;
        if ((Asgn == Bsgn) && (Asgn != tempsgn))
            psw |= PSW_O;
        if (temp > UINT16_MAX)
            psw |= PSW_C;
        if (tempsgn)
            psw |= PSW_N;
    }
        break;
    case OC_NOT:
        C_ = ~A_;
        break;
    case OC_AND:
        C_ = A_ & B_;
        break;
    case OC_OR:
        C_ = A_ | B_;
        break;
    case OC_XOR:
        C_ = A_ ^ B_;
        break;
    case OC_TEST: {
        psw &= ~(PSW_Z | PSW_N);
        ushort temp = A_ & B_;
        if (temp == 0)
            psw |= PSW_Z;
        if (temp & USHORT_SIGN_BIT)
            psw |= PSW_N;
    }
        break;
    case OC_SHL:
        psw &= ~(PSW_Z | PSW_C | PSW_N);
        if ((A_ << (B_ - 1u)) & USHORT_SIGN_BIT)
            psw |= PSW_C;
        C_ = A_ << B_;
        if (C_ == 0)
            psw |= PSW_Z;
        if (C_ & USHORT_SIGN_BIT)
            psw |= PSW_N;
        break;
    case OC_SHR:
        psw &= ~(PSW_Z | PSW_C | PSW_N);
        if (((int16_t)A_ >> ((int16_t)B_ - 1)) & 1)
            psw |= PSW_C;
        C_ = (int16_t)A_ >> (int16_t)B_;
        if (C_ == 0)
            psw |= PSW_Z;
        if (C_ & USHORT_SIGN_BIT)
            psw |= PSW_N;
        break;
    case OC_LDR:
        C_ = B_;
        break;
    case OC_STR:
        C_ = A_;
        break;
    }

    return EE_OK;
}

int Emulator::writeback()
{
    switch (IR_InstrDescr_OC(ir_)) {
    case OC_XCHG:
        registers_[IR_RegsDescr_RD(ir_)] = B_;
        registers_[IR_RegsDescr_RS(ir_)] = A_;
        break;
    case OC_GROUP_ARITHMETICAL:
    case OC_GROUP_LOGICAL:
    case OC_GROUP_SHIFT:
    case OC_LDR:
        registers_[IR_RegsDescr_RD(ir_)] = C_;
        break;
    case OC_STR:
        switch (IR_AddrMode_AM(ir_)) {
        case AM_REGDIR:
            registers_[IR_RegsDescr_RS(ir_)] = C_;
            break;
        case AM_REGDIR_OFFSET:
            registers_[IR_RegsDescr_RS(ir_)] = C_ + IR_Data(ir_);
            break;
        case AM_REGIND:
        case AM_REGIND_OFFSET:
        case AM_MEMDIR:
            writeWord(B_, C_);
            break;
        break;
        }
    }

    return EE_OK;
}

void Emulator::intr()
{
    ushort& psw = registers_[REG_PSW];

    ubyte irqNo;
    bool accepted = false;
    for (irqNo = 0; irqNo < IVT_SIZE; irqNo++) {
        if (!interruptQuery((IRQNo)irqNo))
            continue;
        
        if (irqNo == IRQ_RESET) { // nmi
            terminate();
            init(inFilename_);
            return;
        } else if (irqNo == IRQ_USAGE_FAULT) { // nmi
            accepted = true;
            break;
        } else { // maskable interrupts
            if (psw & PSW_I)
                break;
            if (irqNo == IRQ_TIMER && psw & PSW_Tr)
                continue;
            if (irqNo == IRQ_TERMINAL && psw & PSW_Tl)
                continue;
            accepted = true;
            break;
        }
    }
    if (!accepted)
        return;

    interruptClear((IRQNo)irqNo);
    
    ushort& pc = registers_[REG_PC];

    push(psw);
    push(pc);

    psw |= PSW_I; // mask interrupts
    pc = readWord(IRQ_ADDR(irqNo));
}

void Emulator::interruptRequest(IRQNo irqNo)
{
    irq_ |= (1u << irqNo);
}
bool Emulator::interruptQuery(IRQNo irqNo)
{
    return irq_ & (1u << irqNo);
}
void Emulator::interruptClear(IRQNo irqNo)
{
    irq_ &= ~(1u << irqNo);
}

void Emulator::push(ushort val)
{
    ushort& sp = registers_[REG_SP];
    sp -= 2;
    writeWord(sp, val);
}
ushort Emulator::pop()
{
    ushort& sp = registers_[REG_SP];
    ushort val = readWord(sp);
    sp += 2;
    return val;
}

void Emulator::preUpdateRegister(ushort& reg)
{
    switch (IR_AddrMode_Up(ir_)) {
    case REGIND_PRE_INC:
        reg += 2;
        break;
    case REGIND_PRE_DEC:
        reg -= 2;
        break;
    }
}
void Emulator::postUpdateRegister(ushort& reg)
{
    switch (IR_AddrMode_Up(ir_)) {
    case REGIND_POST_INC:
        reg += 2;
        break;
    case REGIND_POST_DEC:
        reg -= 2;
        break;
    }
}

ushort Emulator::readWord(ushort addr) const
{
    ushort val = 0;

    if (addr < MMAP_REGION_START)
        return *((ushort*)&memory_[addr]);
    else { // memory mapped register address
        switch (addr) {
        case MREG_TERM_IN:
            val = termIn_;
            break;
        case MREG_TERM_OUT:
            val = termOut_;
            break;
        case MREG_TIM_CFG:
            val = timCfg_;
            break;
        default:
            // interruptRequest(IVT_USAGE_FAULT);
            break;
        }
    }

    return val;
}

void Emulator::writeWord(ushort addr, ushort val)
{
    if (addr < MMAP_REGION_START)
        *((ushort*)&memory_[addr]) = val;
    else { // memory mapped register address
        switch (addr) {
        case MREG_TERM_IN:
            termIn_ = val;
            break;
        case MREG_TERM_OUT:
            termOut_ = val;
            fputc((char)val, stdout); // term_out -> terminal
            break;
        case MREG_TIM_CFG:
            timCfg_ = val;
            interval_ = TIM_CFG_INTERVALS[timCfg_];
            break;
        default:
            // interruptRequest(IVT_USAGE_FAULT);
            break;
        }
    }
}
