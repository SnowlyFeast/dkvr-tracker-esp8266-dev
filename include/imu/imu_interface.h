#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/dkvr_types.h"

#if defined(DKVR_IMU_CONTROL) || (defined(II_NAME) && (DKVR_IMU_INTERFACE_IMPLEMENTER == II_NAME))
#define LINKAGE
#else
#define LINKAGE static
#endif

#ifdef __cplusplus
extern "C" {
#endif

// it's already defined in imu_control.c, do not define them again
extern int dkvr_imu_data_ready;

// implement required
LINKAGE dkvr_err internal_dkvr_imu_init();
LINKAGE dkvr_err internal_dkvr_imu_read(float* gyr_out, float* acc_out, float* mag_out);
LINKAGE dkvr_err internal_dkvr_imu_handle_interrupt();
LINKAGE const struct dkvr_hardware_specification* internal_dkvr_imu_get_spec();

#ifdef __cplusplus
}
#endif