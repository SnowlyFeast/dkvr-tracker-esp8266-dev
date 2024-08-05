#include "tracker/calibration.h"

#include <string.h>

#define NOISE_VARIANCE_BIT  0b001
#define MAG_REFERENCE_BIT   0b010

static int noise_var_updated = 0;
static int mag_ref_updated = 0;
static float gyr_transform[12];
static float acc_transform[12];
static float mag_transform[12];
static float noise_var[9];
static float magnetic_reference[3];

static void apply_transform(float* src, float* matrix);


void tracker_calibration_transform(float* gyr, float* acc, float* mag)
{
    apply_transform(gyr, gyr_transform);
    apply_transform(acc, acc_transform);
    apply_transform(mag, mag_transform);
}

int tracker_calibration_is_noise_variance_updated()
{
    int temp = noise_var_updated;
    noise_var_updated = 0;
    return temp;
}
int tracker_calibration_is_mag_reference_updated()
{
    int temp = mag_ref_updated;
    mag_ref_updated = 0;
    return temp;
}

const float* tracker_calibration_get_noise_variance() { return noise_var; }
const float* tracker_calibration_get_magnetic_reference() { return magnetic_reference; }

void tracker_calibration_reset()
{
    // reset
    for (int i = 0; i < 12; i++)
    {
        gyr_transform[i] = 0.0f;
        acc_transform[i] = 0.0f;
        mag_transform[i] = 0.0f;
    }

    // set identity
    for (int i = 0; i < 3; i++)
    {
        gyr_transform[i * 5] = 1.0f;
        acc_transform[i * 5] = 1.0f;
        mag_transform[i * 5] = 1.0f;
    }

    // noise var and magnetic reference
    for (int i = 0; i < 3; i++)
    {
        noise_var[i] = 0.0f;
        noise_var[i + 3] = 0.0f;
        noise_var[i + 6] = 0.0f;
        magnetic_reference[i] = 0.0f;
    }
}

void tracker_calibration_set_gyr_transform(const float* new_gyr) { memcpy(gyr_transform, new_gyr, sizeof(gyr_transform)); }
void tracker_calibration_set_acc_transform(const float* new_acc) { memcpy(acc_transform, new_acc, sizeof(acc_transform)); }
void tracker_calibration_set_mag_transform(const float* new_mag) { memcpy(mag_transform, new_mag, sizeof(mag_transform)); }

void tracker_calibration_set_noise_variance(const float *new_noise_var) 
{
    if (new_noise_var[0] == 0.0f)
        return;
    memcpy(noise_var, new_noise_var, sizeof(noise_var));
    noise_var_updated = 1;
}
void tracker_calibration_set_mag_reference(const float *mag_ref)
{
    if (mag_ref[0] == 0.0f)
        return;
    memcpy(magnetic_reference, mag_ref, sizeof(magnetic_reference));
    mag_ref_updated = 1;
}

static void apply_transform(float* src, float* matrix)
{
    float result[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
            result[i] += src[j] * matrix[i * 4 + j];
        result[i] += matrix[i * 4 + 3];
    }
    memcpy(src, result, sizeof(result));
}