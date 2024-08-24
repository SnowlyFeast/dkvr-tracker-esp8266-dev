#define DKVR_IMU_CONTROL
#include "imu/imu_control.h"
#include "imu/imu_interface.h"

#include "common/dkvr_core.h"

int dkvr_imu_data_ready = 0;
struct dkvr_imu_readings dkvr_imu_raw = {0};

int dkvr_imu_is_data_ready()
{
    int temp = dkvr_imu_data_ready;
    dkvr_imu_data_ready = 0;
    return temp;
}

dkvr_err dkvr_imu_init()
{
    return internal_dkvr_imu_init();
}

dkvr_err dkvr_imu_read()
{
    dkvr_err res = internal_dkvr_imu_read(dkvr_imu_raw.gyr, dkvr_imu_raw.acc, dkvr_imu_raw.mag);
    if (res != DKVR_OK) return res;

    // fix orientation
    float x, y, z;
    x = dkvr_imu_raw.gyr[0];
    y = dkvr_imu_raw.gyr[1];
    z = dkvr_imu_raw.gyr[2];
    dkvr_imu_raw.gyr[0] = DKVR_REAL_GYRO_X;
    dkvr_imu_raw.gyr[1] = DKVR_REAL_GYRO_Y;
    dkvr_imu_raw.gyr[2] = DKVR_REAL_GYRO_Z;
    
    x = dkvr_imu_raw.acc[0];
    y = dkvr_imu_raw.acc[1];
    z = dkvr_imu_raw.acc[2];
    dkvr_imu_raw.acc[0] = DKVR_REAL_ACCEL_X;
    dkvr_imu_raw.acc[1] = DKVR_REAL_ACCEL_Y;
    dkvr_imu_raw.acc[2] = DKVR_REAL_ACCEL_Z;
    
    x = dkvr_imu_raw.mag[0];
    y = dkvr_imu_raw.mag[1];
    z = dkvr_imu_raw.mag[2];
    dkvr_imu_raw.mag[0] = DKVR_REAL_MAG_X;
    dkvr_imu_raw.mag[1] = DKVR_REAL_MAG_Y;
    dkvr_imu_raw.mag[2] = DKVR_REAL_MAG_Z;

    return DKVR_OK;
}

dkvr_err dkvr_imu_handle_interrupt()
{
    return internal_dkvr_imu_handle_interrupt();
}

const struct dkvr_hardware_specification* dkvr_imu_get_spec()
{
    return internal_dkvr_imu_get_spec();
}