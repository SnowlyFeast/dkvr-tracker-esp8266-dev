#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct dkvr_imu_readings
{
    float gyr[3], acc[3], mag[3];
};

extern struct dkvr_imu_readings dkvr_imu_raw;

// the flag clears to 0 after function call
int dkvr_imu_is_data_ready();

dkvr_err dkvr_imu_init();
dkvr_err dkvr_imu_read();
dkvr_err dkvr_imu_handle_interrupt();
const struct dkvr_hardware_specification* dkvr_imu_get_spec();

#ifdef __cplusplus
}
#endif