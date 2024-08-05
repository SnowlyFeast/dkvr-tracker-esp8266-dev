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


typedef struct mpu6050_handle handle;
typedef struct mpu6050_configuration configuration;

static inline uint8_t mpu6050_read(handle* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return hptr->i2c_read(hptr->config.address, reg, len, buffer, 0);
}

static inline uint8_t mpu6050_write(handle* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return hptr->i2c_write(hptr->config.address, reg, len, buffer, 0);
}

static inline uint8_t update_configuration_key(handle* hptr, configuration* newconf, size_t offset, uint8_t reg)
{
    uint8_t value = ((uint8_t*)newconf)[offset];
    uint8_t* target = (uint8_t*)&hptr->config + offset;
    if (*target != value)
    {
        uint8_t err = mpu6050_write(hptr, reg, 1, &value);
        if (err) return err;
        *target = value;
    }
    return 0;
}

static inline float gyro_lsb_resolution(uint8_t gyro_fsr)
{
    switch (gyro_fsr)
    {
        case MPU6050_GYRO_FSR_250_BIT:
            return 7.629e-3f;
        case MPU6050_GYRO_FSR_500_BIT:
            return 1.526e-2f;
        case MPU6050_GYRO_FSR_1000_BIT:
            return 3.052e-2f;
        case MPU6050_GYRO_FSR_2000_BIT:
            return 6.104e-2f;
        default:
            return 0;
    }
}

static inline float accel_lsb_resolution(uint8_t accel_fsr)
{
    switch (accel_fsr)
    {
        case MPU6050_ACCEL_FSR_2_BIT:
            return 6.104e-5f;
        case MPU6050_ACCEL_FSR_4_BIT:
            return 1.221e-4f;
        case MPU6050_ACCEL_FSR_8_BIT:
            return 2.441e-4f;
        case MPU6050_ACCEL_FSR_16_BIT:
            return 4.883e-4f;
        default:
            return 0;
    }
}

void mpu6050_set_address(struct mpu6050_configuration* config, mpu6050_address address) { config->address = address; }
void mpu6050_set_clksel(struct mpu6050_configuration* config, mpu6050_clksel clksel) { config->clksel = clksel; }
void mpu6050_set_sampling_rate(struct mpu6050_configuration* config, uint8_t sampling_rate) { config->smplrt = sampling_rate; }
void mpu6050_set_dlpf(struct mpu6050_configuration* config, mpu6050_dlpf dlpf) { config->dlpf = dlpf; }
void mpu6050_set_gyro_fsr(struct mpu6050_configuration* config, mpu6050_gyro_fsr gyro_fsr) { config->gyro_fsr = gyro_fsr; }
void mpu6050_set_accel_fsr(struct mpu6050_configuration* config, mpu6050_accel_fsr accel_fsr) { config->accel_fsr = accel_fsr; }

void mpu6050_enable_temp_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x80; }
void mpu6050_enable_gyro_x_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x40; }
void mpu6050_enable_gyro_y_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x20; }
void mpu6050_enable_gyro_z_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x10; }
void mpu6050_enable_accel_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x08; }
void mpu6050_enable_slv_2_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x04; }
void mpu6050_enable_slv_1_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x02; }
void mpu6050_enable_slv_0_fifo(struct mpu6050_configuration* config) { config->fifo_en |= 0x01; }

void mpu6050_enable_int_active_low(struct mpu6050_configuration* config) { config->int_conf |= 0x80; }
void mpu6050_enable_int_open_drain(struct mpu6050_configuration* config) { config->int_conf |= 0x40; }
void mpu6050_enable_latch_int_en(struct mpu6050_configuration* config) { config->int_conf |= 0x20; }
void mpu6050_enable_int_rd_clr(struct mpu6050_configuration* config) { config->int_conf |= 0x10; }
void mpu6050_enable_i2c_bypass_en(struct mpu6050_configuration* config) { config->int_conf |= 0x02; }

void mpu6050_enable_fifo_oflow_en(struct mpu6050_configuration* config) { config->int_enable |= 0x10; }
void mpu6050_enable_i2c_mst_int_en(struct mpu6050_configuration* config) { config->int_enable |= 0x08; }
void mpu6050_enable_data_rdy_en(struct mpu6050_configuration* config) { config->int_enable |= 0x01; }

void mpu6050_enable_fifo_en(struct mpu6050_configuration* config) { config->user_ctrl |= 0x40; }
void mpu6050_enable_i2c_mst_en(struct mpu6050_configuration* config) { config->user_ctrl |= 0x20; }
void mpu6050_enable_i2c_if_dis(struct mpu6050_configuration* config) { config->user_ctrl |= 0x10; }
void mpu6050_enable_fifo_reset(struct mpu6050_configuration* config) { config->user_ctrl |= 0x04; }
void mpu6050_enable_i2c_mst_reset(struct mpu6050_configuration* config) { config->user_ctrl |= 0x02; }
void mpu6050_enable_sig_cond_reset(struct mpu6050_configuration* config) { config->user_ctrl |= 0x01; }

int mpu6050_is_fifo_oflow_interrupt(mpu6050_interrupt interrupt) { return (interrupt & 0x10);}
int mpu6050_is_i2c_mst_interrupt(mpu6050_interrupt interrupt) { return (interrupt & 0x08); }
int mpu6050_is_data_rdy_interrupt(mpu6050_interrupt interrupt) { return (interrupt & 0x01); }


mpu6050_handle_test_result mpu6050_test_handle(struct mpu6050_handle* hptr)
{
    // check callback
    if (!hptr->i2c_read || !hptr->i2c_write || !hptr->delay)
        return MPU6050_HANDLE_MISSING_CALLBACK;
    
    // test R/W
    uint8_t temp;
    if (mpu6050_read(hptr, MPU6050_REG_CONFIG, 1, &temp))
        return MPU6050_HANDLE_I2C_READ_FAILED;
    if (mpu6050_write(hptr, MPU6050_REG_CONFIG, 1, &temp))
        return MPU6050_HANDLE_I2C_WRITE_FAILED;
    
    return MPU6050_HANDLE_OK;
}

mpu6050_self_test_result mpu6050_run_self_test(struct mpu6050_handle* hptr)
{
    // reset
    uint8_t err = mpu6050_reset(hptr);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;

    // configure self test
    struct mpu6050_configuration self_test_config =
    {
        .clksel = MPU6050_CLK_GYRO_Z_BIT,
        .smplrt_div = 9,
        .dlpf = MPU6050_DLPF_42_HZ_BIT,
        .gyro_fsr = MPU6050_GYRO_SELFTEST_BIT | MPU6050_GYRO_FSR_250_BIT,
        .accel_fsr = MPU6050_ACCEL_SELFTEST_BIT | MPU6050_ACCEL_FSR_8_BIT
    };
    err = mpu6050_configure(hptr, &self_test_config);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;

    hptr->delay(50);

    // record self test value
    float gyro_selftest[3], accel_selftest[3];
    err |= mpu6050_read_gyro(hptr, gyro_selftest);
    err |= mpu6050_read_accel(hptr, accel_selftest);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;

    // disable self test
    self_test_config.gyro_fsr = MPU6050_GYRO_FSR_250_BIT;
    self_test_config.accel_fsr = MPU6050_ACCEL_FSR_8_BIT;
    err = mpu6050_configure(hptr, &self_test_config);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;

    hptr->delay(50);

    // record raw value
    float gyro_raw[3], accel_raw[3];
    err |= mpu6050_read_gyro(hptr, gyro_raw);
    err |= mpu6050_read_accel(hptr, accel_raw);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;


    // get test value
    /* |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
     * -----------------------------------------------------------------
     * |      XA_TEST[4-2]     |              XG_TEST[4-0]             |
     * |      YA_TEST[4-2]     |              YG_TEST[4-0]             |
     * |      ZA_TEST[4-2]     |              ZG_TEST[4-0]             |
     * |   RESERVED    | XA_TEST[1-0]  | YA_TEST[1-0]  | ZA_TEST[1-0]  |
     * ----------------------------------------------------------------*/
    uint8_t buffer[4];
    err = mpu6050_read(hptr, MPU6050_REG_SELF_TEST_X, 4, buffer);
    if (err) return MPU6050_SELF_TEST_I2C_ERROR;

    uint8_t gyro_test[3], accel_test[3];
    for (int i = 0; i < 3; i++) 
    {
        gyro_test[i]  =   buffer[i]       & 0x1F;
        accel_test[i] = ((buffer[i] >> 3) & 0x1C) | ((buffer[3]) >> (4 - i * 2) & 0x03);
    }

    // calculate factory trim(FT)
    double gyro_ft[3], accel_ft[3];
    for (int i = 0; i < 3; i++) 
    {
        gyro_ft[i]  = gyro_test[i]  ? (25.0 * 131.0  * pow(1.046, gyro_test[i] - 1.0)) * (i % 2 ? -1 : 1) : 0.0;
        accel_ft[i] = accel_test[i] ? (4096.0 * 0.34 * pow((0.92 / 0.34), ((accel_test[i] - 1.0) / 30.0))) : 0.0;
    }

    // caluclate Self-Test Response
    for (int i = 0; i < 3; i++)
    {
        gyro_selftest[i] -= gyro_raw[i];
        accel_selftest[i] -= accel_raw[i];
        gyro_ft[i] = fabs(gyro_selftest[i] / gyro_ft[i] - 1.0);
        accel_ft[i] = fabs(accel_selftest[i] / accel_ft[i] - 1.0);
    }

    // create result
    uint8_t result = MPU6050_SELF_TEST_PASSED;
    for (int i = 0; i < 3; i++) 
    {
        if (gyro_ft[i] > 0.14)
            result |= MPU6050_SELF_TEST_GYRO_FAILED;
        
        if (accel_ft[i] > 0.14)
            result |= MPU6050_SELF_TEST_ACCEL_FAILED;
    }

    return result;
}

uint8_t mpu6050_reset(struct mpu6050_handle* hptr)
{
    uint8_t data = MPU6050_DEVICE_RESET_BIT;
    uint8_t err = mpu6050_write(hptr, MPU6050_REG_PWR_MGMT_1, 1, &data);
    if (err) return err;
    
    hptr->delay(100);

    err |= mpu6050_read(hptr, MPU6050_REG_PWR_MGMT_1, 1, &hptr->config.clksel);
    err |= mpu6050_read(hptr, MPU6050_REG_SMPRT_DIV, 1, &hptr->config.smplrt_div);
    err |= mpu6050_read(hptr, MPU6050_REG_CONFIG, 1, &hptr->config.dlpf);
    err |= mpu6050_read(hptr, MPU6050_REG_GYRO_CONFIG, 1, &hptr->config.gyro_fsr);
    err |= mpu6050_read(hptr, MPU6050_REG_ACCEL_CONFIG, 1, &hptr->config.accel_fsr);
    err |= mpu6050_read(hptr, MPU6050_REG_FIFO_EN, 1, &hptr->config.fifo_en);
    err |= mpu6050_read(hptr, MPU6050_REG_INT_PIN_CFG, 1, &hptr->config.int_conf);
    err |= mpu6050_read(hptr, MPU6050_REG_INT_ENABLE, 1, &hptr->config.int_enable);
    err |= mpu6050_read(hptr, MPU6050_REG_USER_CTRL, 1, &hptr->config.user_ctrl);
    
    return err;
}

uint8_t mpu6050_configure(struct mpu6050_handle* hptr, struct mpu6050_configuration* new_config)
{
    // calculate smplrt_div
    if (new_config->smplrt)
    {
        uint16_t fs = new_config->dlpf ? 1000 : 8000;
        new_config->smplrt_div = (uint8_t)((fs / new_config->smplrt) - 1);
    }

    uint8_t err = 0;
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, clksel), MPU6050_REG_PWR_MGMT_1);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, smplrt_div), MPU6050_REG_SMPRT_DIV);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, dlpf), MPU6050_REG_CONFIG);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, gyro_fsr), MPU6050_REG_GYRO_CONFIG);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, accel_fsr), MPU6050_REG_ACCEL_CONFIG);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, fifo_en), MPU6050_REG_FIFO_EN);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, int_conf), MPU6050_REG_INT_PIN_CFG);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, int_enable), MPU6050_REG_INT_ENABLE);
    err |= update_configuration_key(hptr, new_config, offsetof(configuration, user_ctrl), MPU6050_REG_USER_CTRL);

    return err;
}

uint8_t mpu6050_enable_bypass(struct mpu6050_handle* hptr)
{
    // this process does not overwrite the handle's configuration
    uint8_t err = 0;
    uint8_t data = 0x00;
    err |= mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &data);
    
    data = 0x02;
    err |= mpu6050_write(hptr, MPU6050_REG_INT_PIN_CFG, 1, &data);

    return err;
}

uint8_t mpu6050_disable_bypass(struct mpu6050_handle* hptr)
{
    // restore config with the handle's saved configuration
    uint8_t err = 0;
    err |= mpu6050_write(hptr, MPU6050_REG_INT_PIN_CFG, 1, &hptr->config.int_conf);
    err |= mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &hptr->config.user_ctrl);
    return err;
}

uint8_t mpu6050_enable_external(struct mpu6050_handle* hptr)
{
    uint8_t err = 0;

    // write address
    uint8_t data = hptr->config.ext_addr | MPU6050_I2C_SLV0_R_MODE_BIT;
    err |= mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_ADDR, 1, &data);

    // write target reg
    data = hptr->config.ext_reg;
    err |= mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_REG, 1, &data);

    // enable slv 0
    data = MPU6050_I2C_SLV0_EN_BIT | hptr->config.ext_len;
    err |= mpu6050_write(hptr, MPU6050_REG_I2C_SLV0_CTRL, 1, &data);

    // syncronization setting
    data = MPU6050_WAIT_FOR_ES_BIT | MPU6050_I2C_MST_CLK_400_BIT;
    err |= mpu6050_write(hptr, MPU6050_REG_I2C_MST_CTRL, 1, &data);

    // enable master mode
    mpu6050_enable_i2c_mst_en(&hptr->config);
    err |= mpu6050_write(hptr, MPU6050_REG_USER_CTRL, 1, &hptr->config.user_ctrl);
    
    return err;
}

uint8_t mpu6050_read_interrupt(struct mpu6050_handle* hptr, mpu6050_interrupt* int_out)
{
    return mpu6050_read(hptr, MPU6050_REG_INT_STATUS, 1, int_out);
}

uint8_t mpu6050_read_gyro(struct mpu6050_handle* hptr, float* gyr_out)
{
    uint8_t buffer[6];
    uint8_t err = mpu6050_read(hptr, MPU6050_REG_GYRO_XOUT_H, 6, buffer);
    if (err) return err;

    float resolution = gyro_lsb_resolution(hptr->config.gyro_fsr);
    gyr_out[0] = (float)(((int16_t)buffer[0] << 8) | buffer[1]) * resolution;
    gyr_out[1] = (float)(((int16_t)buffer[2] << 8) | buffer[3]) * resolution;
    gyr_out[2] = (float)(((int16_t)buffer[4] << 8) | buffer[5]) * resolution;

    return 0;
}

uint8_t mpu6050_read_accel(struct mpu6050_handle* hptr, float* acc_out)
{
    uint8_t buffer[6];
    uint8_t err = mpu6050_read(hptr, MPU6050_REG_ACCEL_XOUT_H, 6, buffer);
    if (err) return err;
    
    float resolution = accel_lsb_resolution(hptr->config.accel_fsr);
    acc_out[0] = (float)(((int16_t)buffer[0] << 8) | buffer[1]) * resolution;
    acc_out[1] = (float)(((int16_t)buffer[2] << 8) | buffer[3]) * resolution;
    acc_out[2] = (float)(((int16_t)buffer[4] << 8) | buffer[5]) * resolution;
    
    return 0;
}

uint8_t mpu6050_read_external(struct mpu6050_handle* hptr, uint8_t* ext_out)
{
    return mpu6050_read(hptr, MPU6050_REG_EXT_SENS_DATA, hptr->config.ext_len, ext_out);
}
