#include "driver/qmc5883l_driver.h"

#include <assert.h>
#include <string.h>

// Register address
#define QMC5883L_REG_CONTROL_1                  0x09    // Control Register 1
#define QMC5883L_REG_CONTROL_2                  0x0A    // Control Register 2
#define QMC5883L_REG_PERIOD_FBR                 0x0B    // SET/RESET Preiod Register

// Configuration Const
#define QMC5883L_SOFT_RESET_BIT                 0x80
#define QMC5883L_PERIOD_FBR_VAL                 0x01

// configuration bit mask
#define MASK_CONTROL1_MODE                      0b00000011
#define MASK_CONTROL1_ODR                       0b00001100
#define MASK_CONTROL1_RNG                       0b00110000
#define MASK_CONTROL1_OSR                       0b11000000
#define MASK_CONTROL2_INT_EN                    0b00000001
#define MASK_CONTROL2_ROL_PNT                   0b01000000


typedef struct qmc5883l_handle handle;
typedef struct qmc5883l_configuration configuration;

static inline uint8_t qmc5883l_read(handle* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return hptr->i2c_read(QMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0);
}

static inline uint8_t qmc5883l_write(handle* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return hptr->i2c_write(QMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0);
}

static inline float mag_lsb_resolution(uint8_t control1)
{
    return (control1 & QMC5883L_RNG_8G) ? 2.441e-4f : 6.104e-5f;
}


void qmc5883l_set_mode(struct qmc5883l_configuration* config, qmc5883l_mode mode)
{
    config->control1 &= ~MASK_CONTROL1_MODE;
    config->control1 |= mode;
}

void qmc5883l_set_output_data_rate(struct qmc5883l_configuration* config, qmc5883l_odr odr)
{
    config->control1 &= ~MASK_CONTROL1_ODR;
    config->control1 |= odr;
}

void qmc5883l_set_scale_range(struct qmc5883l_configuration* config, qmc5883l_rng rng)
{
    config->control1 &= ~MASK_CONTROL1_RNG;
    config->control1 |= rng;
}

void qmc5883l_set_over_sample_rate(struct qmc5883l_configuration* config, qmc5883l_osr osr)
{
    config->control1 &= ~MASK_CONTROL1_OSR;
    config->control1 |= osr;
}

void qmc5883l_set_interrupt(struct qmc5883l_configuration* config, int enable)
{
    enable ? (config->control2 |= MASK_CONTROL2_INT_EN) : (config->control2 &= ~MASK_CONTROL2_INT_EN);
}

void qmc5883l_set_pointer_rollover(struct qmc5883l_configuration* config, int enable)
{
    enable ? (config->control2 |= MASK_CONTROL2_ROL_PNT) : (config->control2 &= ~MASK_CONTROL2_ROL_PNT);
}

qmc5883l_handle_test_result qmc5883l_test_handle(struct qmc5883l_handle* hptr)
{
    // check callback
    if (!hptr->i2c_read || !hptr->i2c_write || !hptr->delay)
        return QMC5883L_HANDLE_MISSING_CALLBACK;
    
    // test R/W
    uint8_t temp;
    if (qmc5883l_read(hptr, QMC5883L_REG_PERIOD_FBR, 1, &temp))
        return QMC5883L_HANDLE_I2C_READ_FAILED;
    if (qmc5883l_write(hptr, QMC5883L_REG_PERIOD_FBR, 1, &temp))
        return QMC5883L_HANDLE_I2C_WRITE_FAILED;
    
    return QMC5883L_HANDLE_OK;
}

uint8_t qmc5883l_reset(struct qmc5883l_handle* hptr)
{
    // soft reset
    uint8_t data = QMC5883L_SOFT_RESET_BIT;
    uint8_t err = qmc5883l_write(hptr, QMC5883L_REG_CONTROL_2, 1, &data);
    if (err) return err;

    hptr->delay(50);

    // set fbr (this value was given by hardware specification, dun know what it is)
    data = QMC5883L_PERIOD_FBR_VAL;
    err |= qmc5883l_write(hptr, QMC5883L_REG_PERIOD_FBR, 1, &data);

    // sync configuration
    err |= qmc5883l_read(hptr, QMC5883L_REG_CONTROL_1, 1, &hptr->config.control1);
    err |= qmc5883l_read(hptr, QMC5883L_REG_CONTROL_2, 1, &hptr->config.control2);

    return err;
}

uint8_t qmc5883l_configure(struct qmc5883l_handle* hptr, const struct qmc5883l_configuration new_conf)
{
    if (hptr->config.control1 != new_conf.control1) {
        int err = qmc5883l_write(hptr, QMC5883L_REG_CONTROL_1, 1, &new_conf.control1);
        if (err) return err;
        hptr->config.control1 = new_conf.control1;
    }

    if (hptr->config.control2 != new_conf.control2) {
        int err = qmc5883l_write(hptr, QMC5883L_REG_CONTROL_2, 1, &new_conf.control2);
        if (err) return err;
        hptr->config.control2 = new_conf.control2;
    }

    return 0;
}

uint8_t qmc5883l_read_mag(struct qmc5883l_handle* hptr, float* mag_out)
{
    uint8_t buffer[6];
    uint8_t err = qmc5883l_read(hptr, QMC5883L_REG_DATA_BEGIN, 6, buffer);
    if (err) return err;

    float resolution = mag_lsb_resolution(hptr->config.control1);
    mag_out[0] = (float)(((int16_t)buffer[1] << 8) | buffer[0]) * resolution;
    mag_out[1] = (float)(((int16_t)buffer[3] << 8) | buffer[2]) * resolution;
    mag_out[2] = (float)(((int16_t)buffer[5] << 8) | buffer[4]) * resolution;

    return 0;
}

void qmc5883l_convert_mag_from_external(struct qmc5883l_handle* hptr, uint8_t* ext_read, float* mag_out)
{
    float resolution = mag_lsb_resolution(hptr->config.control1);
    mag_out[0] = (float)(((int16_t)ext_read[1] << 8) | ext_read[0]) * resolution;
    mag_out[1] = (float)(((int16_t)ext_read[3] << 8) | ext_read[2]) * resolution;
    mag_out[2] = (float)(((int16_t)ext_read[5] << 8) | ext_read[4]) * resolution;
}
