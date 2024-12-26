#pragma once

#define DKVR_TRANSFORM_MATRIX_SIZE  48
#define DKVR_NOISE_VARIANCE_SIZE    36

#ifdef __cplusplus
extern "C" {
#endif

void tracker_calib_transform_readings(float* gyr, float* acc, float* mag);

int  tracker_calib_is_noise_variance_updated();
void tracker_calib_get_noise_variance(float* gyr, float* acc, float* mag);

void tracker_calib_reset();
void tracker_calib_set_scale_factor(float scale);
void tracker_calib_set_gyr_transform(const float* new_gyr);
void tracker_calib_set_acc_transform(const float* new_acc);
void tracker_calib_set_mag_transform(const float* new_mag);
void tracker_calib_set_noise_variance(const float* new_noise_var);

#ifdef __cplusplus
}
#endif