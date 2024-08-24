#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tracker_calibration_transform(float* gyr, float* acc, float* mag);

int tracker_calibration_is_noise_variance_updated();
void tracker_calibration_get_noise_variance(float* gyr, float* acc, float* mag);

void tracker_calibration_reset();
void tracker_calibration_set_gyr_transform(const float* new_gyr);
void tracker_calibration_set_acc_transform(const float* new_acc);
void tracker_calibration_set_mag_transform(const float* new_mag);
void tracker_calibration_set_noise_variance(const float* new_noise_var);

#ifdef __cplusplus
}
#endif