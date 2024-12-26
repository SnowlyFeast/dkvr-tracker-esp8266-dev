#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tracker_statistic
{
    uint8_t execution_time;         // execution time of update() loop
    uint8_t interrupt_miss_rate;    // percentage of unhandled interrupt
    uint8_t imu_miss_rate;          // percentage of IMU read failure
};

const struct tracker_statistic* tracker_statistic_get_struct_ptr();

void tracker_statistic_record_cycle_end();
void tracker_statistic_record_interrupt_miss();
void tracker_statistic_record_imu_miss();

#ifdef __cplusplus
}
#endif