#include "tracker/tracker_config.h"

#include <string.h>

#include "common/dkvr_core.h"

#define BEHAVIOR_ACTIVE_BIT_MASK    (0b00000001)
#define BEHAVIOR_RAW_BIT_MASK       (0b00000010)
#define BEHAVIOR_LED_BIT_MASK       (0b00000100)

#define DOT_PRODUCT(lhs, rhs)       (lhs->x * rhs->x + lhs->y * rhs->y + lhs->z * rhs->z)


static tracker_behavior_t behavior = 0;
static float gyro_calib_offset[3] = {};     // offset vector
static float accel_calib_mat[12]  = {};     // transposed
static float mag_calib_mat[12]    = {};     // transposed

static void calibrate_gyro(vector3_t* gyro);
static void calibrate_accel(vector3_t* accel);
static void calibrate_mag(vector3_t* mag);


void reset_tracker_config()
{
    static const float calib_reset_table[] PROGMEM
    = { 0, 0, 0,       // 0
        1, 1, 1, 0,    // 3
        1, 1, 1, 0,
        1, 1, 1, 0,
        1, 1, 1, 0,    // 15
        1, 1, 1, 0,
        1, 1, 1, 0 };

    behavior = BEHAVIOR_LED_BIT_MASK;
    memcpy_P(gyro_calib_offset, calib_reset_table + 0 , sizeof(gyro_calib_offset));
    memcpy_P(accel_calib_mat,   calib_reset_table + 3 , sizeof(accel_calib_mat));
    memcpy_P(mag_calib_mat,     calib_reset_table + 15, sizeof(mag_calib_mat));
}

tracker_behavior_t get_behavior()
{
    return behavior;
}

int get_behavior_active()
{
    return (behavior & BEHAVIOR_ACTIVE_BIT_MASK);
}

int get_behavior_raw()
{
    return (behavior & BEHAVIOR_RAW_BIT_MASK);
}

int get_behavior_led()
{
    return (behavior & BEHAVIOR_LED_BIT_MASK);
}

void calibrate_imu(vector3_t* gyro, vector3_t* accel, vector3_t* mag)
{
    calibrate_gyro(gyro);
    calibrate_accel(accel);
    calibrate_mag(mag);
}

void set_behavior(uint8_t new_behavior)
{
    behavior = new_behavior;
}

void set_behavior_active()
{
    behavior |= BEHAVIOR_ACTIVE_BIT_MASK;
}

void set_behavior_inactive()
{
    behavior &= ~BEHAVIOR_ACTIVE_BIT_MASK;
}

void set_behavior_raw(int on)
{
    if (on)
        behavior |= BEHAVIOR_RAW_BIT_MASK;
    else
        behavior &= ~BEHAVIOR_RAW_BIT_MASK;
}

void set_behavior_led(int on)
{
    if (on)
        behavior |= BEHAVIOR_LED_BIT_MASK;
    else
        behavior &= ~BEHAVIOR_LED_BIT_MASK;
}

void set_gyro_offset(const float* gyro_offset)
{
    memcpy(gyro_calib_offset, gyro_offset, sizeof(gyro_calib_offset));
}

void set_accel_calib_mat(const float* accel_mat)
{
    memcpy(accel_calib_mat, accel_mat, sizeof(accel_calib_mat));
}

void set_mag_calib_mat(const float* mag_mat)
{
    memcpy(mag_calib_mat, mag_mat, sizeof(mag_calib_mat));
}

static void calibrate_gyro(vector3_t* gyro)
{
    gyro->x += gyro_calib_offset[0];
    gyro->y += gyro_calib_offset[1];
    gyro->z += gyro_calib_offset[2];
}

static void calibrate_accel(vector3_t* accel)
{
    vector3_t* col1 = (vector3_t*)&accel_calib_mat[0];
    vector3_t* col2 = (vector3_t*)&accel_calib_mat[4];
    vector3_t* col3 = (vector3_t*)&accel_calib_mat[8];

    float new_x = DOT_PRODUCT(accel, col1) + accel_calib_mat[3];
    float new_y = DOT_PRODUCT(accel, col2) + accel_calib_mat[7];
    float new_z = DOT_PRODUCT(accel, col3) + accel_calib_mat[11];

    accel->x = new_x;
    accel->y = new_y;
    accel->z = new_z;
}

static void calibrate_mag(vector3_t* mag)
{
    vector3_t* col1 = (vector3_t*)&mag_calib_mat[0];
    vector3_t* col2 = (vector3_t*)&mag_calib_mat[4];
    vector3_t* col3 = (vector3_t*)&mag_calib_mat[8];

    float new_x = DOT_PRODUCT(mag, col1) + mag_calib_mat[3];
    float new_y = DOT_PRODUCT(mag, col2) + mag_calib_mat[7];
    float new_z = DOT_PRODUCT(mag, col3) + mag_calib_mat[11];
    
    mag->x = new_x;
    mag->y = new_y;
    mag->z = new_z;
}