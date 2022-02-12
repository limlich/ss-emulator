#ifndef REGISTER_H
#define REGISTER_H

#include "types.hpp"

enum Register: ubyte
{
    REG_SP = 6u,
    REG_PC = 7u,
    REG_PSW = 8u,
    NUM_REGISTERS
};

enum PSWBitMask: ushort
{
    PSW_BIT_Z = 1u << 0,
    PSW_BIT_O = 1u << 1,
    PSW_BIT_C = 1u << 2,
    PSW_BIT_N = 1u << 3,
    PSW_BIT_Tr = 1u << 13, // timer mask
    PSW_BIT_Tl = 1u << 14, // terminal mask
    PSW_BIT_I = 1u << 15   // exti mask
};

#define PSW_Z(psw) ((bool)((psw) & PSW_BIT_Z))
#define PSW_O(psw) ((bool)((psw) & PSW_BIT_O))
#define PSW_C(psw) ((bool)((psw) & PSW_BIT_C))
#define PSW_N(psw) ((bool)((psw) & PSW_BIT_N))
#define PSW_Tr(psw) ((bool)((psw) & PSW_BIT_Tr))
#define PSW_Tl(psw) ((bool)((psw) & PSW_BIT_Tl))
#define PSW_I(psw) ((bool)((psw) & PSW_BIT_I))

#define MMAP_REGION_START 0xFF00u
#define INITIAL_SP MMAP_REGION_START
#define INITIAL_PSW 0x00u

enum MMapRegister: ushort
{
    MREG_TERM_OUT = 0xFF00,
    MREG_TERM_IN = 0xFF02,
    MREG_TIM_CFG = 0xFF10
};

enum RegIndUpdateType: ubyte
{
    REGIND_NONE = 0x00u,
    REGIND_PRE_DEC = 0x01u,
    REGIND_PRE_INC = 0x02u,
    REGIND_POST_DEC = 0x03u,
    REGIND_POST_INC = 0x04u
};

#endif
