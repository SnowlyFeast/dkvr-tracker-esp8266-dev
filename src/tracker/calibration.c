#include "tracker/calibration.h"

#include <string.h>

#include "common/dkvr_core.h"

#include "imu/imu_control.h"

static int noise_var_updated = 0;
static float gyr_transform[12];
static float acc_transform[12];
static float mag_transform[12];
static float noise_var[9];
static float scale_factor = 1.0f;

// factory calibration
static const float gyr_reset[12] PROGMEM  = { DKVR_IMU_FACTORY_CALIBRATION_GYRO };
static const float acc_reset[12] PROGMEM  = { DKVR_IMU_FACTORY_CALIBRATION_ACCEL };
static const float mag_reset[12] PROGMEM  = { DKVR_IMU_FACTORY_CALIBRATION_MAG };
#ifdef DKVR_IMU_FACTORY_CALIBRATION_NOISE_VAR
static const float noise_reset[9] PROGMEM = { DKVR_IMU_FACTORY_CALIBRATION_NOISE_VAR };
#endif

static inline int is_zero_array(const float* arr) { return arr[0] == 0.0f && arr[1] == 0.0f && arr[2] == 0.0f; }
static inline void reset_gyr_transform()  { memcpy_P(gyr_transform, gyr_reset,   sizeof(gyr_transform)); }
static inline void reset_acc_transform()  { memcpy_P(acc_transform, acc_reset,   sizeof(acc_transform)); }
static inline void reset_mag_transform()  { memcpy_P(mag_transform, mag_reset,   sizeof(mag_transform)); }
static inline void reset_noise_variance() 
{ 
#ifdef DKVR_IMU_FACTORY_CALIBRATION_NOISE_VAR
    memcpy_P(noise_var,     noise_reset, sizeof(noise_var)); 
#else
    memcpy(noise_var + 0, dkvr_imu_get_spec()->noise_variance_gyr, sizeof(float) * 3);
    memcpy(noise_var + 3, dkvr_imu_get_spec()->noise_variance_acc, sizeof(float) * 3);
    memcpy(noise_var + 6, dkvr_imu_get_spec()->noise_variance_mag, sizeof(float) * 3);
#endif
}
static void apply_transform(float* src, float* matrix);

void tracker_calib_transform_readings(float* gyr, float* acc, float* mag)
{
    apply_transform(gyr, gyr_transform);
    apply_transform(acc, acc_transform);
    apply_transform(mag, mag_transform);

#ifdef DKVR_IMU_MAG_BOOT_TIME_SCALING
    mag[0] *= scale_factor;
    mag[1] *= scale_factor;
    mag[2] *= scale_factor;
#endif
}

int tracker_calib_is_noise_variance_updated()
{
    int temp = noise_var_updated;
    noise_var_updated = 0;
    return temp;
}

void tracker_calib_get_noise_variance(float* gyr, float* acc, float* mag)
{
    memcpy(gyr, noise_var + 0, sizeof(float) * 3);
    memcpy(acc, noise_var + 3, sizeof(float) * 3);
    memcpy(mag, noise_var + 6, sizeof(float) * 3);
}

void tracker_calib_reset()
{
    reset_gyr_transform();
    reset_acc_transform();
    reset_mag_transform();
    reset_noise_variance();
}

void tracker_calib_set_scale_factor(float scale) { scale_factor = scale; }

void tracker_calib_set_gyr_transform(const float* new_gyr) 
{ 
    if (is_zero_array(new_gyr))
        reset_gyr_transform();
    else
        memcpy(gyr_transform, new_gyr, sizeof(gyr_transform));
}

void tracker_calib_set_acc_transform(const float* new_acc) 
{ 
    if (is_zero_array(new_acc))
        reset_acc_transform();
    else
        memcpy(acc_transform, new_acc, sizeof(acc_transform));
}

void tracker_calib_set_mag_transform(const float* new_mag) 
{ 
    if (is_zero_array(new_mag))
        reset_mag_transform();
    else
        memcpy(mag_transform, new_mag, sizeof(mag_transform));
}

void tracker_calib_set_noise_variance(const float *new_noise_var) 
{
    if (is_zero_array(new_noise_var))
        reset_noise_variance();
    else
        memcpy(noise_var, new_noise_var, sizeof(noise_var));

    noise_var_updated = 1;
}

static void apply_transform(float* src, float* matrix)
{
    // calibration matrix is column-major
    float result[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            result[j] += src[i] * matrix[i * 3 + j];
        result[i] += matrix[9 + i];
    }
    memcpy(src, result, sizeof(result));
}