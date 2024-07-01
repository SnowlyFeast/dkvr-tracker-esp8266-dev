#include "imu/configured/configured_mpu6050.h"

#include <string.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/dkvr_types.h"
#include "common/i2c_interface.h"
#include "common/system_interface.h"

#include "driver/mpu6050_driver.h"

mpu6050_handle_t mpu6050_configured_handle = {};

// just damn lazy short named pointer of the configured handle. worthless.
static mpu6050_handle_t* hptr = &mpu6050_configured_handle;

static float gyro_lsb_resolution = 0;
static float accel_lsb_resolution = 0;

dkvr_err_t init_configured_mpu6050(int a0)
{
    memset(hptr, 0, sizeof(mpu6050_handle_t));
    hptr->i2c_read = dkvr_i2c_read;
    hptr->i2c_write = dkvr_i2c_write;
    hptr->delay = dkvr_delay;

    mpu6050_set_address(hptr, a0 ? MPU6050_DEVICE_ADDRESS_A0_HIGH : MPU6050_DEVICE_ADDRESS_A0_LOW);
    mpu6050_result_t mpu_result = mpu6050_assert_handle(hptr);
    if (mpu_result == MPU6050_I2C_RW_FAIL)
        return DKVR_ERR_GYRO_INIT_FAIL;

    mpu6050_conf_t conf;
    memset(&conf, 0, sizeof(conf));

    mpu6050_fifo_en_t fifo_en = 0;
    mpu6050_int_conf_t int_conf = 0;
    mpu6050_int_en_t int_en = 0;
    MPU6050_ENABLE_I2C_MST_INT_EN(int_en);
    MPU6050_ENABLE_DATA_RDY_EN(int_en);

    mpu6050_set_clksel(&conf, MPU6050_CLK_GYRO_Z_BIT);
    mpu6050_set_sampling_rate(&conf, DKVR_IMU_SAMPLING_RATE);
    mpu6050_set_dlpf(&conf, MPU6050_DLPF_42_HZ_BIT);
    mpu6050_set_gyro_fsr(&conf, MPU6050_GYRO_FSR_1000_BIT);
    mpu6050_set_accel_fsr(&conf, MPU6050_ACCEL_FSR_2_BIT);
    mpu6050_set_fifo_en(&conf, fifo_en);
    mpu6050_set_int_conf(&conf, int_conf);
    mpu6050_set_int_en(&conf, int_en);

    if (mpu6050_reset(hptr))
        return DKVR_ERR_GYRO_INIT_FAIL;
    
    if (mpu6050_configure(hptr, &conf))
        return DKVR_ERR_GYRO_INIT_FAIL;

    gyro_lsb_resolution = MPU6050_GYRO_LSB_RESOLUTION(hptr->conf.gyro_fsr);
    accel_lsb_resolution = MPU6050_ACCEL_LSB_RESOLUTION(hptr->conf.accel_fsr);
    
    return DKVR_OK;
}

dkvr_err_t read_interrupt_configured_mpu6050(mpu6050_interrupt_t* int_out)
{
    mpu6050_result_t result = mpu6050_read_intterrupt(hptr, int_out);

    return (result == MPU6050_OK) ? DKVR_OK : DKVR_ERR_INT_READ_FAIL;
}

dkvr_err_t read_gyro_configured_mpu6050(vector3_t* gyro_out)
{
    mpu6050_vec3s_t gyro = {};
    mpu6050_result_t result = mpu6050_read_gyro(hptr, &gyro);
    if (result != MPU6050_OK)
        return DKVR_ERR_SENSOR_READ_FAIL;
    
    // fix orientation
    float x = gyro.x * gyro_lsb_resolution;
    float y = gyro.y * gyro_lsb_resolution;
    float z = gyro.z * gyro_lsb_resolution;
    gyro_out->x = DKVR_REAL_GYRO_X;
    gyro_out->y = DKVR_REAL_GYRO_Y;
    gyro_out->z = DKVR_REAL_GYRO_Z;

    return DKVR_OK;
}

dkvr_err_t read_accel_configured_mpu6050(vector3_t* accel_out)
{
    mpu6050_vec3s_t accel = {};
    mpu6050_result_t result = mpu6050_read_accel(hptr, &accel);
    if (result != MPU6050_OK)
        return DKVR_ERR_SENSOR_READ_FAIL;

    // fix orientation
    float x = accel.x * accel_lsb_resolution;
    float y = accel.y * accel_lsb_resolution;
    float z = accel.z * accel_lsb_resolution;
    accel_out->x = DKVR_REAL_ACCEL_X;
    accel_out->y = DKVR_REAL_ACCEL_Y;
    accel_out->z = DKVR_REAL_ACCEL_Z;
    
    return DKVR_OK;
}

dkvr_err_t read_external_configured_mpu6050(uint64_t* ext_out)
{
    mpu6050_result_t result = mpu6050_read_external(hptr, (uint8_t*)ext_out);
    if (result != MPU6050_OK)
        return DKVR_ERR_SENSOR_READ_FAIL;
        
    return DKVR_OK;
}