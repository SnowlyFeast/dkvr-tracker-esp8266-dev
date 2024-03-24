#include "imu/imu_control.h"

#include "common/dkvr_core.h"
#include "common/dkvr_hardware_id.h"

// import hardware 1 handle
#if (DKVR_HARDWARE_ID_1 == DVKR_HARDWARE_ID_MPU6050)
#   define  HW1_MPU6050
#   include "driver/mpu6050_driver.h"
#   include "imu/configured/configured_mpu6050.h"
#else
#   error hardware 1 not specified
#endif

// import hardware 2 handle
#if (DKVR_HARDWARE_ID_2 == DKVR_HARDWARE_ID_HMC5883L)
#   define  HW2_HMC5883L
#   include "driver/hmc5883l_driver.h"
#   include "imu/configured/configured_hmc5883l.h"
#elif (DKVR_HARDWARE_ID_2 == DKVR_HARDWARE_ID_QMC5883L)
#   define HW2_QMC5883L
#   include "driver/qmc5883l_driver.h"
#   include "imu/configured/configured_qmc5883l.h"
#else
#   error hardware 2 not specified
#endif

// result assertion macro
#define ASSERT_RESULT(func, ...)               \
    do                                         \
    {                                          \
        dkvr_err_t result = func(__VA_ARGS__); \
        if (result != DKVR_OK)                 \
            return result;                     \
    } while (0)

// imu sensor readings output variable
imu_raw_t imu_raw = {};

static int imu_data_ready = 0;

int is_imu_data_ready()
{
    int temp = imu_data_ready;
    imu_data_ready = 0;
    return temp;
}

dkvr_err_t init_imu()
{
#if defined(HW1_MPU6050)
    ASSERT_RESULT(init_configured_mpu6050, MPU6050_A0_LOW);
    mpu6050_enable_bypass(&mpu6050_configured_handle);
    mpu6050_external_t external = {};

    #if defined(HW2_HMC5883L)
    {
        ASSERT_RESULT(init_configured_hmc5883l);
        external.addr = HMC5883L_DEVICE_ADDRESS;
        external.reg = HMC5883L_REG_DATA_BEGIN;
        external.len = HMC5883L_DATA_LENGTH;
    }
    #elif defined(HW2_QMC5883L)
    {
        ASSERT_RESULT(init_configured_qmc5883l);
        external.addr = QMC5883L_DEVICE_ADDRESS;
        external.reg = QMC5883L_REG_DATA_BEGIN;
        external.len = QMC5883L_DATA_LENGTH;
    }
    #else
    #   error not implemented hw combination
    #endif
    
    #ifdef DKVR_HARDWARE_ENABLE_MPU6050_MASTER
    {
        mpu6050_disable_bypass(&mpu6050_configured_handle);
        mpu6050_setup_external(&mpu6050_configured_handle, external);
    }
    #endif

    return DKVR_OK;
#else
#   error not implemented hw1 init
#endif
}

dkvr_err_t handle_interrupt()
{
#if defined(HW1_MPU6050)
    mpu6050_interrupt_t int_out;
    ASSERT_RESULT(read_interrupt_configured_mpu6050, &int_out);

    if (MPU6050_IS_INT_FIFO_OFLOW(int_out))
    {
        // not gonna happen, cuz it's not enabled
    }

    if (MPU6050_IS_INT_I2C_MST(int_out))
    {
        // also not enabled, but maybe someday
    }

    if (MPU6050_IS_INT_DATA_RDY(int_out))
    {
        imu_data_ready = 1;
    }

    return DKVR_OK;
#else
#   error not implemented hw1 interrupt handler
#endif
}

dkvr_err_t update_imu_readings()
{
#if defined(HW1_MPU6050)
    ASSERT_RESULT(read_gyro_configured_mpu6050, &imu_raw.gyro_out);
    ASSERT_RESULT(read_accel_configured_mpu6050, &imu_raw.accel_out);

    uint64_t buffer;
    uint64_t* bptr = NULL;
    #ifdef DKVR_HARDWARE_ENABLE_MPU6050_MASTER
    {
        ASSERT_RESULT(read_external_configured_mpu6050, &buffer);
        bptr = &buffer;
    }
    #endif

    #if defined(HW2_HMC5883L)
    {
        ASSERT_RESULT(read_mag_configured_hmc5883l, &imu_raw.mag_out, bptr);
    }
    #elif defined(HW2_QMC5883L)
    {
        ASSERT_RESULT(read_mag_configured_qmc5883l, &imu_raw.mag_out, bptr);
    }
    #else
    #   error not implemented hw combination
    #endif

    return DKVR_OK;
#else
#   error not implemented hw1 read imu function
#endif
}