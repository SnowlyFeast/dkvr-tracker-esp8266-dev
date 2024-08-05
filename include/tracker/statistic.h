#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tracker_statistic
{
    uint8_t execution_time;
    uint8_t interrupt_miss_rate;
    uint8_t imu_miss_rate;
};

const struct tracker_statistic* tracker_statistic_get_struct_ptr();

void tracker_statistic_record_execution_time();
void tracker_statistic_record_interrupt_miss();
void tracker_statistic_record_imu_miss();

#ifdef __cplusplus
}
#endif