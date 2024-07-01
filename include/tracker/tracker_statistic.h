#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct tracker_statistic_s
{
    uint8_t execution_time;
} tracker_statistic_t;

tracker_statistic_t get_tracker_statistic();

void record_execution_time();    

#ifdef __cplusplus
}
#endif