#ifndef IVT_H
#define IVT_H

#include "types.hpp"

enum IVTEntry: ubyte
{
    IVT_RESET = 0u,
    IVT_USAGE_FAULT = 1u,
    IVT_TIMER = 2u,
    IVT_TERMINAL = 3u,
    IVT_IRQ0 = 4u,
    IVT_IRQ1 = 5u,
    IVT_IRQ2 = 6u,
    IVT_IRQ3 = 7u,
    IVT_SIZE
};

#define IVT_START 0x0000u
#define IVT_OFFSET(ivtEntry) (IVT_START + ((ivtEntry) << 2u))

#endif
