#ifndef INSTRUCTION_H
#define INSTRUCTION_H

#include <unordered_map>

#include "types.hpp"

typedef ubyte addr_mode_type;

enum AddrMode: addr_mode_type
{
    AM_IMMED = 0b0000u,
    AM_REGDIR = 0b0001u,
    AM_REGDIR_OFFSET = 0b0101u,
    AM_REGIND = 0b0010u,
    AM_REGIND_OFFSET = 0b0011u,
    AM_MEMDIR = 0b0100u
};

enum Opcode : ubyte
{
    OC_HALT = 0x00u,
    OC_INT  = 0x10u,
    OC_IRET = 0x20u,
    OC_CALL = 0x30u,
    OC_RET  = 0x40u,
    OC_JMP  = 0x50u,
    OC_JEQ  = 0x51u,
    OC_JNE  = 0x52u,
    OC_JGT  = 0x53u,
    OC_XCHG = 0x60u,
    OC_ADD  = 0x70u,
    OC_SUB  = 0x71u,
    OC_MUL  = 0x72u,
    OC_DIV  = 0x73u,
    OC_CMP  = 0x74u,
    OC_NOT  = 0x80u,
    OC_AND  = 0x81u,
    OC_OR   = 0x82u,
    OC_XOR  = 0x83u,
    OC_TEST = 0x84u,
    OC_SHL  = 0x90u,
    OC_SHR  = 0x91u,
    OC_LDR  = 0xA0u,
    OC_STR  = 0xB0u,
    OC_PUSH = 0xB0u,
    OC_POP  = 0xA0u,
};

#endif
