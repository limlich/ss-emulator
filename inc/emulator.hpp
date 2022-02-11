#ifndef EMULATOR_H
#define EMULATOR_H

#include <fstream>
#include <string>
#include <vector>
#include <termios.h>

#include "register.hpp"
#include "ivt.hpp"
#include "timer.hpp"
#include "types.hpp"

enum EmulatorExitCode: int
{
    EE_OK = 0,
    EE_FILE = 1,
    EE_OPCODE = 2,
    EE_ADDR_MODE = 3,
    EE_REG = 4
};

#define MEMORY_SIZE 0x10000 // 64kB

class Emulator
{
public:
    Emulator();

    int run(const std::string& inFilename);

private:
    int init(const std::string& inFilename);
    void terminate();
    int loop();

    void handleInput();
    void updateTimer();
    void instr();
    int fetch();
    int decode();
    int execute();
    int writeback();
    void intr();

    void interruptRequest(IRQNo irqNo);
    bool interruptQuery(IRQNo irqNo);
    void interruptClear(IRQNo irqNo);

    void push(ushort val);
    ushort pop();

    ushort readWord(ushort addr) const;
    void writeWord(ushort addr, ushort val);

private:
    std::string inFilename_;

    termios tOldStdin_;
    bool running_;

    std::vector<ubyte> memory_;
    ushort registers_[NUM_REGISTERS]; // r0-7, psw
    // memory mapped registers
    ushort termIn_;
    ushort termOut_;
    ushort timCfg_;

    ubyte ir_[5]; // instruction register

    // timer
    std::chrono::high_resolution_clock::time_point t0_;
    uint interval_;

    // interrupt requests
    ubyte irq_;
};

#endif
