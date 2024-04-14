#include "driver/mpu6050_driver.h"

#include <math.h>
#include <string.h>

// Register address
#define MPU6050_REG_PWR_MGMT_1      0x6B    // Power Management 1
#define MPU6050_REG_SELF_TEST_X     0x0D    // Self Test Register
#define MPU6050_REG_SMPRT_DIV       0x19    // Sample Rate Divider
#define MPU6050_REG_CONFIG          0x1A    // DLPF Configuration
#define MPU6050_REG_GYRO_CONFIG     0x1B    // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG    0x1C    // Accelerometer Configuration
#define MPU6050_REG_FIFO_EN         0x23    // FIFO Enable
#define MPU6050_REG_INT_PIN_CFG     0x37    // INT Pin / Bypass Enable Configuration
#define MPU6050_REG_INT_ENABLE      0x38    // Interrupt Enable
#define MPU6050_REG_INT_STATUS      0x3A    // Interrupt Status
#define MPU6050_REG_USER_CTRL       0x6A    // User Control
#define MPU6050_REG_I2C_MST_CTRL    0x24    // I2C Master Control

#define MPU6050_REG_I2C_SLV0_ADDR   0x25    // I2C Slave 0
#define MPU6050_REG_I2C_SLV0_REG    0x26
#define MPU6050_REG_I2C_SLV0_CTRL   0x27

#define MPU6050_REG_ACCEL_XOUT_H    0x3B    // Raw Data Out
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48

#define MPU6050_REG_EXT_SENS_DATA   0x49    // External Sensor Data Out
#define MPU6050_REG_I2C_SLV0_DO     0x63

// MPU6050 Configuration Const
#define MPU6050_DEVICE_RESET_BIT    0x80    // Soft Reset Bit
#define MPU6050_GYRO_SELFTEST_BIT   0xE0    // Gyro Selftest Bit
#define MPU6050_ACCEL_SELFTEST_BIT  0xE0    // Accel Selftest Bit

#define MPU6050_WAIT_FOR_ES_BIT     0x40    // I2C Master Control Bit
#define MPU6050_I2C_MST_CLK_400_BIT 0x0D

#define MPU6050_I2C_SLV0_R_MODE_BIT 0x80    // I2C Slave 0 Control
#define MPU6050_I2C_SLV0_EN_BIT     0x80


#define ASSERT_RESULT(hptr)         if ((hptr)->i2c_result) return (hptr)->i2c_result

#define MPU6050_READ_CONFIG(hptr, key, reg)              \
    do                                                   \
    {                                                    \
        mpu6050_read(hptr, reg, 1, &((hptr)->conf.key)); \
        ASSERT_RESULT(hptr);                             \
    } while (0);

#define MPU6050_UPDATE_CONFIG(hptr, cptr, key, reg)    \
    do                                                 \
    {                                                  \
        if ((hptr)->conf.key != (cptr)->key)           \
        {                                              \
            mpu6050_write(hptr, reg, 1, &(cptr)->key); \
            ASSERT_RESULT(hptr);                       \
            (hptr)->conf.key = (cptr)->key;            \
        }                                              \
    } while (0)

static inline uint8_t mpu6050_read(mpu6050_handle_t* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_read(hptr->conf.address, reg, len, buffer, 0));
}

static inline uint8_t mpu6050_write(mpu6050_handle_t* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return (hptr->i2c_result = hptr->i2c_write(hptr->conf.address, reg, len, buffer, 0));
}

void mpu6050_set_address(mpu6050_handle_t* hptr, mpu6050_address_t address)
{
    hptr->conf.address = address;
}

void mpu6050_set_clksel(mpu6050_conf_t* cptr, mpu6050_clksel_t clksel)
{
    cptr->clksel = clksel;
}

void mpu6050_set_sampling_rate(mpu6050_conf_t* cptr, uint8_t smplrt)
{
    cptr->smplrt = smplrt;
}

void mpu6050_set_dlpf(mpu6050_conf_t* cptr, mpu6050_dlpf_t dlpf)
{
    cptr->dlpf = dlpf;
}

void mpu6050_set_gyro_fsr(mpu6050_conf_t* cptr, mpu6050_gyro_fsr_t gyro_fsr)
{
    cptr->gyro_fsr = gyro_fsr;
}

void mpu6050_set_accel_fsr(mpu6050_conf_t* cptr, mpu6050_accel_fsr_t accel_fsr)
{
    cptr->accel_fsr = accel_fsr;
}

void mpu6050_set_fifo_en(mpu6050_conf_t* cptr, mpu6050_fifo_en_t fifo_en)
{
    cptr->fifo_en = fifo_en;
}

void mpu6050_set_int_conf(mpu6050_conf_t* cptr, mpu6050_int_conf_t int_conf)
{
    cptr->int_conf = int_conf;
}

void mpu6050_set_int_en(mpu6050_conf_t* cptr, mpu6050_int_en_t int_en)
{
    cptr->int_enable = int_en;
}

void mpu6050_attach_i2c_read(mpu6050_handle_t* hptr, mpu6050_i2c_read_callback callback)
{
    hptr->i2c_read = callback;
}

void mpu6050_attach_i2c_write(mpu6050_handle_t* hptr, mpu6050_i2c_write_callback callback)
{
    hptr->i2c_write = callback;
}

void mpu6050_attach_delay(mpu6050_handle_t *hptr, mpu6050_delay_callback callback)
{
    hptr->delay = callback;
}

mpu6050_result_t mpu6050_assert_handle(mpu6050_handle_t *hptr)
{
    mpu6050_result_t result = MPU6050_OK;
    if (!hptr->i2c_read)
        result |= MPU6050_MISSING_I2C_R_CB;
    if (!hptr->i2c_write)
        result |= MPU6050_MISSING_I2C_W_CB;
    if (!hptr->delay)
        result |= MPU6050_MISSING_DELAY_CB;

    // incomplete handle
    if (result)
        return result;
    
    // test i2c read / write
    uint8_t temp;
    if (mpu6050_read(hptr, MPU6050_REG_CONFIG, 1, &temp))
        return MPU6050_I2C_RW_FAIL;
    if (mpu6050_write(hptr, MPU6050_REG_CONFIG, 1, &temp))
        return MPU6050_I2C_RW_FAIL;
    
    return MPU6050_OK;
}

uint8_t mpu6050_reset(mpu6050_handle_t* hptr)
{
    uint8_t data = MPU6050_DEVICE_RESET_BIT;
    mpu6050_write(hptr, MPU6050_REG_PWR_MGMT_1, 1, &data);
    ASSERT_RESULT(hptr);
    hptr->delay(100);

    MPU6050_READ_CONFIG(hptr, clksel,       MPU6050_REG_PWR_MGMT_1);
    MPU6050_READ_CONFIG(hptr, smplrt_div,   MPU6050_REG_SMPRT_DIV);
    MPU6050_READ_CONFIG(hptr, dlpf,         MPU6050_REG_CONFIG);
    MPU6050_READ_CONFIG(hptr, gyro_fsr,     MPU6050_REG_GYRO_CONFIG);
    MPU6050_READ_CONFIG(hptr, accel_fsr,    MPU6050_REG_ACCEL_CONFIG);
    MPU6050_READ_CONFIG(hptr, fifo_en,      MPU6050_REG_FIFO_EN);
    MPU6050_READ_CONFIG(hptr, int_conf,     MPU6050_REG_INT_PIN_CFG);
    MPU6050_READ_CONFIG(hptr, int_enable,   MPU6050_REG_INT_ENABLE);
    MPU6050_READ_CONFIG(hptr, user_ctrl,    MPU6050_REG_USER_CTRL);
    
    return 0;
}

uint8_t mpu6050_configure(mpu6050_handle_t* hptr, mpu6050_conf_t* cptr)
{
    // calculate smplrt_div
    if (cptr->smplrt)
    {
        uint16_t fs = cptr->dlpf ? 1000 : 8000;
        cptr->smplrt_div = (uint8_t)((fs / cptr->smplrt) - 1);
    }

    MPU6050_UPDATE_CONFIG(hptr, cptr, clksel,       MPU6050_REG_PWR_MGMT_1);
    MPU6050_UPDATE_CONFIG(hptr, cptr, smplrt_div,   MPU6050_REG_SMPRT_DIV);
    MPU6050_UPDATE_CONFIG(hptr, cptr, dlpf,         MPU6050_REG_CONFIG);
    MPU6050_UPDATE_CONFIG(hptr, cptr, gyro_fsr,     MPU6050_REG_GYRO_CONFIG);
    MPU6050_UPDATE_CONFIG(hptr, cptr, accel_fsr,    MPU6050_REG_ACCEL_CONFIG);
    MPU6050_UPDATE_CONFIG(hptr, cptr, fifo_en,      MPU6050_REG_FIFO_EN);
    MPU6050_UPDATE_CONFIG(hptr, cptr, int_conf,     MPU6050_REG_INT_PIN_CFG);
    MPU6050_UPDATE_CONFIG(hptr, cptr, int_enable,   MPU6050_REG_INT_ENABLE);
    MPU6050_UPDATE_CONFIG(hptr, cptr, user_ctrl,    MPU6050_REG_USER_CTRL);

    return 0;
}

mpu6050_result_t mpu6050_run_self_test(mpu6050_handle_t* hptr)
{
    // reset
    mpu6050_reset(hptr);

    // configure self test
    mpu6050_conf_t conf;
    memset(&conf, 0, sizeof(mpu6050_conf_t));
    conf.clksel = MPU6050_CLK_GYRO_Z_BIT;
    conf.smplrt_div = 9;
    conf.dlpf = MPU6050_DLPF_42_HZ_BIT;
    conf.gyro_fsr = MPU6050_GYRO_SELFTEST_BIT | MPU6050_GYRO_FSR_250_BIT;
    conf.accel_fsr = MPU6050_ACCEL_SELFTEST_BIT | MPU6050_ACCEL_FSR_8_BIT;
    mpu6050_configure(hptr, &conf);
    hptr->delay(50);

    // record self test value
    mpu6050_vec3s_t gyro_selftest, accel_selftest;
    mpu6050_read_gyro(hptr, &gyro_selftest);
    mpu6050_read_accel(hptr, &accel_selftest);

    // disable self test
    conf.gyro_fsr = MPU6050_GYRO_FSR_250_BIT;
    conf.accel_fsr = MPU6050_ACCEL_FSR_8_BIT;
    mpu6050_configure(hptr, &conf);
    hptr->delay(50);

    // record raw value
    mpu6050_vec3s_t gyro_raw, accel_raw;
    mpu6050_read_gyro(hptr, &gyro_raw);
    mpu6050_read_accel(hptr, &accel_raw);

    // calculate factory trim
    uint8_t gyro_test[3], accel_test[3];
    uint8_t buffer[4];
    
    /* |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     * -----------------------------------------------------------------
     * |      XA_TEST[4-2]     |              XG_TEST[4-0]             |
     * |      YA_TEST[4-2]     |              YG_TEST[4-0]             |
     * |      ZA_TEST[4-2]     |              ZG_TEST[4-0]             |
     * |   RESERVED    | XA_TEST[1-0]  | YA_TEST[1-0]  | ZA_TEST[1-0]  |
     * ----------------------------------------------------------------*/
    mpu6050_read(hptr, MPU6050_REG_SELF_TEST_X, 4, buffer);
    for (int i = 0; i < 3; i++) 
    {
        gyro_test[i]  =   buffer[i]       & 0x1F;
        accel_test[i] = ((buffer[i] >> 3) & 0x1C) | ((buffer[3]) >> (4 - i * 2) & 0x03);
    }

    double gyro_ft[3], accel_ft[3];
    for (int i = 0; i < 3; i++) 
    {
        gyro_ft[i]  = gyro_test[i]  ? (25.0 * 131.0  * pow(1.046, gyro_test[i] - 1.0)) * (i % 2 ? -1 : 1) : 0.0;
        accel_ft[i] = accel_test[i] ? (4096.0 * 0.34 * pow((0.92 / 0.34), ((accel_test[i] - 1.0) / 30.0))) : 0.0;
    }

    // Self-Test Response
    gyro_selftest.x -= gyro_raw.x;
    gyro_selftest.y -= gyro_raw.y;
    gyro_selftest.z -= gyro_raw.z;
    accel_selftest.x -= accel_raw.x;
    accel_selftest.y -= accel_raw.y;
    accel_selftest.z -= accel_raw.z;
    gyro_ft[0] = fabs(gyro_selftest.x / gyro_ft[0] - 1.0);
    gyro_ft[1] = fabs(gyro_selftest.y / gyro_ft[1] - 1.0);
    gyro_ft[2] = fabs(gyro_selftest.z / gyro_ft[2] - 1.0);
    accel_ft[0] = fabs(accel_selftest.x / accel_ft[0] - 1.0);
    accel_ft[1] = fabs(accel_selftest.y / accel_ft[1] - 1.0);
    accel_ft[2] = fabs(accel_selftest.z / accel_ft[2] - 1.0);

    // create result
    uint8_t result = 0;
    for (int i = 0; i < 3; i++) 
    {
        if (gyro_ft[i] > 0.14)
            result |= MPU6050_SELF_TEST_FAIL_G;
        
        if (accel_ft[i] > 0.14)
            result |= MPU6050_SELF_TEST_FAIL_A;
    }

    return result;
}

uint8_t mpu6050_enable_bypass(mpu6050_handle_t *hptr)
{
    uint8_t data = 0x00;
    mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &data);
    ASSERT_RESULT(hptr);

    data = 0x02;
    mpu6050_write(hptr, MPU6050_REG_INT_PIN_CFG, 1, &data);
    ASSERT_RESULT(hptr);

    return 0;
}

uint8_t mpu6050_disable_bypass(mpu6050_handle_t *hptr)
{
    mpu6050_write(hptr, MPU6050_REG_INT_PIN_CFG, 1, &hptr->conf.int_conf);
    ASSERT_RESULT(hptr);
    
    mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &hptr->conf.user_ctrl);
    ASSERT_RESULT(hptr);

    return 0;
}

uint8_t mpu6050_setup_external(mpu6050_handle_t *hptr, mpu6050_external_t external)
{
    uint8_t data = external.addr | MPU6050_I2C_SLV0_R_MODE_BIT;
    mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_ADDR, 1, &data);
    ASSERT_RESULT(hptr);

    mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_REG, 1, &external.reg);
    ASSERT_RESULT(hptr);

    data = MPU6050_I2C_SLV0_EN_BIT | external.len;
    mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_CTRL, 1, &data);
    ASSERT_RESULT(hptr);

    data = MPU6050_WAIT_FOR_ES_BIT | MPU6050_I2C_MST_CLK_400_BIT;
    mpu6050_write(hptr, MPU6050_REG_I2C_MST_CTRL, 1, &data);
    ASSERT_RESULT(hptr);

    MPU6050_ENABLE_I2C_MST_EN(hptr->conf.user_ctrl);
    mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &hptr->conf.user_ctrl);
    ASSERT_RESULT(hptr);

    hptr->external = external;
    return 0;
}

uint8_t mpu6050_read_intterrupt(mpu6050_handle_t *hptr, mpu6050_interrupt_t *out)
{
    return mpu6050_read(hptr, MPU6050_REG_INT_STATUS, 1, out);
}

uint8_t mpu6050_read_gyro(mpu6050_handle_t *hptr, mpu6050_vec3s_t *out)
{
    uint8_t buffer[6];
    mpu6050_read(hptr, MPU6050_REG_GYRO_XOUT_H, 6, buffer);
    ASSERT_RESULT(hptr);
    out->x = ((int16_t)buffer[0] << 8) | buffer[1];
    out->y = ((int16_t)buffer[2] << 8) | buffer[3];
    out->z = ((int16_t)buffer[4] << 8) | buffer[5];
    return 0;
}

uint8_t mpu6050_read_accel(mpu6050_handle_t *hptr, mpu6050_vec3s_t *out)
{
    uint8_t buffer[6];
    mpu6050_read(hptr, MPU6050_REG_ACCEL_XOUT_H, 6, buffer);
    ASSERT_RESULT(hptr);
    out->x = ((int16_t)buffer[0] << 8) | buffer[1];
    out->y = ((int16_t)buffer[2] << 8) | buffer[3];
    out->z = ((int16_t)buffer[4] << 8) | buffer[5];
    return 0;
}

uint8_t mpu6050_read_external(mpu6050_handle_t *hptr, uint8_t *out)
{
    mpu6050_read(hptr, MPU6050_REG_EXT_SENS_DATA, hptr->external.len, out);
    ASSERT_RESULT(hptr);
    return 0;
}

float mpu6050_get_gyro_resolution(mpu6050_handle_t* hptr)
{
    return MPU6050_GYRO_LSB_RESOLUTION(hptr->conf.gyro_fsr);
}

float mpu6050_get_accel_resolution(mpu6050_handle_t* hptr)
{
    return MPU6050_ACCEL_LSB_RESOLUTION(hptr->conf.accel_fsr);
}