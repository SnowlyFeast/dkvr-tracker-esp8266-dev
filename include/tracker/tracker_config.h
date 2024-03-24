#pragma once

#include <stdint.h>

#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t tracker_behavior_t;

void reset_tracker_config();

tracker_behavior_t get_behavior();
int get_behavior_active();
int get_behavior_raw();
int get_behavior_led();

void calibrate_imu(vector3_t* gyro, vector3_t* accel, vector3_t* mag);

void set_behavior(uint8_t new_behavior);
void set_behavior_active();
void set_behavior_inactive();
void set_behavior_raw(int on);  // unused
void set_behavior_led(int on);  // unused
void set_gyro_offset(const float* gyro_offset);
void set_accel_calib_mat(const float* accel_mat);
void set_mag_calib_mat(const float* mag_mat);

#ifdef __cplusplus
}
#endif