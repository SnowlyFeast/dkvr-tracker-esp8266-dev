#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tracker_calibration_transform(float* gyr, float* acc, float* mag);

int tracker_calibration_is_noise_variance_updated();
int tracker_calibration_is_mag_reference_updated();
const float* tracker_calibration_get_noise_variance();
const float* tracker_calibration_get_magnetic_reference();

void tracker_calibration_reset();
void tracker_calibration_set_gyr_transform(const float* new_gyr);
void tracker_calibration_set_acc_transform(const float* new_acc);
void tracker_calibration_set_mag_transform(const float* new_mag);
void tracker_calibration_set_noise_variance(const float* new_noise_var);
void tracker_calibration_set_mag_reference(const float* mag_ref);

#ifdef __cplusplus
}
#endif