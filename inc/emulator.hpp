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
    void interruptRequest(IVTEntry ivtEntry);

    ushort readWord(ushort addr) const;
    void writeWord(ushort addr, ushort val);

private:
    termios tOldStdin_;
    bool running_;

    std::vector<ubyte> memory_;
    ushort registers_[NUM_REGISTERS]; // r0-7, psw
    // memory mapped registers
    ushort termIn_;
    ushort termOut_;
    ushort timCfg_;

    // timer
    std::chrono::high_resolution_clock::time_point t0_;
    uint interval_;

    // interrupt requests
    ubyte irq_;
};

#endif
