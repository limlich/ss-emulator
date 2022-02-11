#include "emulator.hpp"

#include <iostream>
#include <unistd.h>

#include "ivt.hpp"

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

    return res;
}

int Emulator::init(const std::string& inFilename)
{
    std::ifstream inFile(inFilename, std::ios_base::in);
    if (!inFile.is_open()) {
        std::cout << "Cannot open file for reading: " << inFilename << std::endl;
        return EE_FILE;
    }
    
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
    registers_[REG_PC] = *((ushort*)&memory_[IVT_OFFSET(IVT_RESET)]);
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
    interruptRequest(IVT_TERMINAL);
}

void Emulator::updateTimer()
{
    auto t1 = std::chrono::high_resolution_clock::now();
    auto elapsed = (t1 - t0_) / std::chrono::milliseconds(1);
    if (elapsed >= interval_) {
        t0_ = std::chrono::high_resolution_clock::now();
        interruptRequest(IVT_TIMER);
    }
}

void Emulator::instr()
{
    // TODO: fetch, decode, execute, intr
}

void Emulator::interruptRequest(IVTEntry ivtEntry)
{
    irq_ |= (1u << ivtEntry);
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
