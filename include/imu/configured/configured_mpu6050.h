#pragma once

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#include "driver/mpu6050_driver.h"

#define MPU6050_A0_HIGH     1
#define MPU6050_A0_LOW      0

#ifdef __cplusplus
extern "C" {
#endif

extern mpu6050_handle_t mpu6050_configured_handle;

dkvr_err init_configured_mpu6050(int a0);
dkvr_err read_interrupt_configured_mpu6050(mpu6050_interrupt_t* int_out);
dkvr_err read_gyro_configured_mpu6050(vector3_t* gyro_out);
dkvr_err read_accel_configured_mpu6050(vector3_t* accel_out);
dkvr_err read_external_configured_mpu6050(uint64_t* ext_out);

#ifdef __cplusplus
}
#endif