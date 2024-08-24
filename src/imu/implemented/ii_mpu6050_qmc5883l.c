#define II_NAME     ii_mpu6050_qmc5883l

#include "imu/imu_interface.h"

#include <string.h>

#include "common/i2c_interface.h"
#include "common/system_interface.h"

#include "driver/mpu6050_driver.h"
#include "driver/qmc5883l_driver.h"

#define ASSERT_RESULT(res)    \
    do                        \
    {                         \
        dkvr_err err = (res); \
        if (err)              \
            return err;       \
    } while (0)

#define EXTERNAL_ADDR       QMC5883L_DEVICE_ADDRESS
#define EXTERNAL_REG        QMC5883L_REG_DATA_BEGIN
#define EXTERNAL_DATA_LEN   QMC5883L_DATA_LENGTH

static struct mpu6050_handle mpu6050;
static struct qmc5883l_handle qmc5883l;

static dkvr_err mpu6050_init();
static dkvr_err mpu6050_handle_interrupt();
static dkvr_err mpu6050_read_gyro_and_accel(float* gyr_out, float* acc_out);
static dkvr_err mpu6050_read_external_mag(float* mag_out);
static dkvr_err qmc5883l_init();

dkvr_err internal_dkvr_imu_init()
{
    ASSERT_RESULT(mpu6050_init());
    ASSERT_RESULT(mpu6050_enable_bypass(&mpu6050));
    ASSERT_RESULT(qmc5883l_init());
    ASSERT_RESULT(mpu6050_disable_bypass(&mpu6050));
    ASSERT_RESULT(mpu6050_enable_external(&mpu6050));
    return DKVR_OK;
}

dkvr_err internal_dkvr_imu_read(float* gyr_out, float* acc_out, float* mag_out)
{
    ASSERT_RESULT(mpu6050_read_gyro_and_accel(gyr_out, acc_out));
    ASSERT_RESULT(mpu6050_read_external_mag(mag_out));
    return DKVR_OK;
}

dkvr_err internal_dkvr_imu_handle_interrupt()
{
    ASSERT_RESULT(mpu6050_handle_interrupt());
    return DKVR_OK;
}

const struct dkvr_hardware_specification* internal_dkvr_imu_get_spec()
{
    static const struct dkvr_hardware_specification spec = {
        .hw1_name = "MPU6050",
        .hw2_name = "QMC5883L",
        .hw3_name = "",
        .noise_variance_gyr = {2.74e-7f, 2.74e-7f, 2.74e-7f},
        .noise_variance_acc = {4.0e-6f, 4.0e-6f, 4.0e-6f},
        .noise_variance_mag = {4.0e-6f, 4.0e-6f, 4.0e-6f}
    };

    return &spec;
}

static dkvr_err mpu6050_init()
{
    // setup handle
    memset(&mpu6050, 0, sizeof(struct mpu6050_handle));
    mpu6050.i2c_read = dkvr_i2c_read;
    mpu6050.i2c_write = dkvr_i2c_write;
    mpu6050.delay = dkvr_delay;

    // test handle
    mpu6050_handle_test_result test = mpu6050_test_handle(&mpu6050);
    switch (test)
    {
    case MPU6050_HANDLE_MISSING_CALLBACK:
        return DKVR_ERR_INCOMPLETE_HANDLE;
    
    case MPU6050_HANDLE_I2C_READ_FAILED:
    case MPU6050_HANDLE_I2C_WRITE_FAILED:
        return DKVR_ERR_GYR_HANDLE_TEST_FAIL;

    default:
        break;
    }

    // run self-test
    mpu6050_self_test_result st = mpu6050_run_self_test(&mpu6050);
    if (st != MPU6050_SELF_TEST_PASSED)
        return DKVR_ERR_GYR_SELF_TEST_FAIL;

    // soft reset
    if (mpu6050_reset(&mpu6050))
        return DKVR_ERR_GYR_INIT_FAIL;

    // configure
    struct mpu6050_configuration config = {0};
    mpu6050_set_clksel(&config, MPU6050_CLK_GYRO_Z_BIT);
    mpu6050_set_sampling_rate(&config, DKVR_IMU_SAMPLING_RATE);
    mpu6050_set_dlpf(&config, MPU6050_DLPF_42_HZ_BIT);
    mpu6050_set_gyro_fsr(&config, MPU6050_GYRO_FSR_2000_BIT);
    mpu6050_set_accel_fsr(&config, MPU6050_ACCEL_FSR_4_BIT);
    mpu6050_enable_i2c_mst_int_en(&config);
    mpu6050_enable_data_rdy_en(&config);
    config.ext_addr = EXTERNAL_ADDR;
    config.ext_reg = EXTERNAL_REG;
    config.ext_len = EXTERNAL_DATA_LEN;

    if (mpu6050_configure(&mpu6050, &config))
        return DKVR_ERR_GYR_INIT_FAIL;
    
    return DKVR_OK;
}

static dkvr_err mpu6050_handle_interrupt()
{
    mpu6050_interrupt interrupt;
    if (mpu6050_read_interrupt(&mpu6050, &interrupt))
        return DKVR_ERR_INT_READ_FAIL;
    
    if (mpu6050_is_data_rdy_interrupt(interrupt))
    {
        dkvr_imu_data_ready = 1;
    }

    if (mpu6050_is_fifo_oflow_interrupt(interrupt))
    {
        // not gonna happen, cuz it's disabled
    }

    if (mpu6050_is_i2c_mst_interrupt(interrupt))
    {
        // same here
    }

    return DKVR_OK;
}

static dkvr_err mpu6050_read_gyro_and_accel(float* gyr_out, float* acc_out)
{
    if (mpu6050_read_gyro(&mpu6050, gyr_out))
        return DKVR_ERR_SENSOR_READ_FAIL;

    if (mpu6050_read_accel(&mpu6050, acc_out))
        return DKVR_ERR_SENSOR_READ_FAIL;

    return DKVR_OK;
}

static dkvr_err mpu6050_read_external_mag(float* mag_out)
{
    uint8_t buffer[EXTERNAL_DATA_LEN];
    if (mpu6050_read_external(&mpu6050, buffer))
        return DKVR_ERR_SENSOR_READ_FAIL;

    hmc5883l_convert_mag_from_external(&qmc5883l, buffer, mag_out);
    
    return DKVR_OK;
}

static dkvr_err qmc5883l_init()
{
    // setup handle
    memset(&qmc5883l, 0, sizeof(struct qmc5883l_handle));
    qmc5883l.i2c_read = dkvr_i2c_read;
    qmc5883l.i2c_write = dkvr_i2c_write;
    qmc5883l.delay = dkvr_delay;

    // test handle
    qmc5883l_handle_test_result test = hmc5883l_test_handle(&qmc5883l);
    switch (test)
    {
    case QMC5883L_HANDLE_MISSING_CALLBACK:
        return DKVR_ERR_INCOMPLETE_HANDLE;
    
    case QMC5883L_HANDLE_I2C_READ_FAILED:
    case QMC5883L_HANDLE_I2C_WRITE_FAILED:
        return DKVR_ERR_MAG_INIT_FAIL;

    default:
        break;
    }

    // configure
    struct qmc5883l_configuration config = {0};
    qmc5883l_set_mode(&config, QMC5883L_MODE_CONTINUOUS);
    qmc5883l_set_output_data_rate(&config, QMC5883L_ODR_100HZ);
    qmc5883l_set_scale_range(&config, QMC5883L_RNG_8G);
    qmc5883l_set_over_sample_rate(&config, QMC5883L_OSR_256);
    qmc5883l_set_interrupt(&config, 1);

    if (qmc5883l_configure(&qmc5883l, config))
        return DKVR_ERR_MAG_INIT_FAIL;

    return DKVR_OK;
}