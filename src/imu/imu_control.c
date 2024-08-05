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
    return internal_dkvr_imu_read(dkvr_imu_raw.gyr, dkvr_imu_raw.acc, dkvr_imu_raw.mag);
}

dkvr_err dkvr_imu_handle_interrupt()
{
    return internal_dkvr_imu_handle_interrupt();
}

const struct dkvr_hardware_specification* dkvr_imu_get_spec()
{
    return internal_dkvr_imu_get_spec();
}