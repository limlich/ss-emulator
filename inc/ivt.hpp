#ifndef IVT_H
#define IVT_H

#include "types.hpp"

enum IRQNo: ubyte
{
    IRQ_RESET = 0u,
    IRQ_USAGE_FAULT = 1u,
    IRQ_TIMER = 2u,
    IRQ_TERMINAL = 3u,
    IRQ_4 = 4u,
    IRQ_5 = 5u,
    IRQ_6 = 6u,
    IRQ_7 = 7u,
    IVT_SIZE
};

#define IVT_START 0x0000u
#define IRQ_ADDR(irqNo) (IVT_START + ((irqNo) << 1u))

#endif
