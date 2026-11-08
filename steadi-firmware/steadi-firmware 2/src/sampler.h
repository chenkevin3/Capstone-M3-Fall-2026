#pragma once
#include <stdint.h>

void sampler_start(void);
// Blocks until the next sample tick; returns the timestamp in microseconds
uint64_t sampler_wait(void);
uint32_t sampler_overruns(void);
