#ifndef TIMER_H
#define TIMER_H

#include <chrono>

#include "types.hpp"

static const uint TIM_CFG_INTERVALS[] = { // in milliseconds
    500u,
    1000u,
    1500u,
    2000u,
    5000u,
    10000u,
    30000u,
    60000u
};

#define INITIAL_TIM_CFG 0
#define TIM_CFG_MAX (sizeof(TIM_CFG_INTERVALS) / sizeof(TIM_CFG_INTERVALS[0]) - 1)


#endif
