#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct imu_raw_s
{
    vector3_t gyro_out;
    vector3_t accel_out;
    vector3_t mag_out;
} imu_raw_t;

extern imu_raw_t imu_raw;

// the flag clears to 0 after the value has been read
int is_imu_data_ready();

dkvr_err_t init_imu();
dkvr_err_t handle_interrupt();
dkvr_err_t update_imu_readings();

#ifdef __cplusplus
}
#endif