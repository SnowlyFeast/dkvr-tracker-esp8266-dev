#include "driver/hmc5883l_driver.h"

#include <string.h>

// Register address
#define HMC5883L_REG_CONF_A                 0x00    // Configuration Register A
#define HMC5883L_REG_CONF_B                 0x01    // Configuration Register B
#define HMC5883L_REG_MODE                   0x02    // Mode Register
#define HMC5883L_REG_STATUS                 0x09    // Status Register

// configuration bit mask
#define MASK_CONFIG_A_MA                    0b01100000
#define MASK_CONFIG_A_DO                    0b00011100
#define MASK_CONFIG_A_MS                    0b00000011
#define MASK_CONFIG_B_GN                    0b11100000
#define MASK_MODE_HS                        0b10000000
#define MASK_MODE_MD                        0b00000011

#define HMC5883L_OVERFLOW                   -4096


typedef struct hmc5883l_handle handle;
typedef struct hmc5883l_configuration config;

static inline uint8_t hmc5883l_read(handle* hptr, uint8_t reg, uint8_t len, uint8_t* buffer)
{
    return hptr->i2c_read(HMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0);
}

static inline uint8_t hmc5883l_write(handle* hptr, uint8_t reg, uint8_t len, const uint8_t* buffer)
{
    return hptr->i2c_write(HMC5883L_DEVICE_ADDRESS, reg, len, buffer, 0);
}

static float mag_lsb_resolution(uint8_t config_b)
{
    switch (config_b & 0xE0)
    {
    case HMC5883L_GN_1370:
        return 7.299e-4f;
    case HMC5883L_GN_1090:
        return 9.174e-4f;
    case HMC5883L_GN_820:
        return 1.220e-3f;
    case HMC5883L_GN_660:
        return 1.515e-3f;
    case HMC5883L_GN_440:
        return 2.273e-3f;
    case HMC5883L_GN_390:
        return 2.564e-3f;
    case HMC5883L_GN_330:
        return 3.030e-3f;
    case HMC5883L_GN_230:
        return 4.348e-3f;
    default:
        return 0;
    }
}


void hmc5883l_set_measurement_averaged(struct hmc5883l_configuration* config, hmc5883l_ma ma)
{
    config->config_a &= ~MASK_CONFIG_A_MA;
    config->config_a |= ma;
}

void hmc5883l_set_data_output_rate(struct hmc5883l_configuration* config, hmc5883l_do dor)
{
    config->config_a &= ~MASK_CONFIG_A_DO;
    config->config_a |= dor;
}

void hmc5883l_set_measurement_configuration(struct hmc5883l_configuration* config, hmc5883l_ms ms)
{
    config->config_a &= ~MASK_CONFIG_A_MS;
    config->config_a |= ms;
}

void hmc5883l_set_gain(struct hmc5883l_configuration* config, hmc5883l_gn gain)
{
    config->config_b &= ~MASK_CONFIG_B_GN;
    config->config_b |= gain;
}

void hmc5883l_set_high_speed_i2c(struct hmc5883l_configuration* config, int enable)
{
    enable ? (config->mode |= MASK_MODE_HS) : (config->mode &= ~MASK_MODE_HS);
}

void hmc5883l_set_operating_mode(struct hmc5883l_configuration* config, hmc5883l_md mode)
{
    config->mode &= ~MASK_MODE_MD;
    config->mode |= mode;
}

hmc5883l_handle_test_result hmc5883l_test_handle(struct hmc5883l_handle* hptr)
{
    // check callback
    if (!hptr->i2c_read || !hptr->i2c_write || !hptr->delay)
        return HMC5883L_HANDLE_MISSING_CALLBACK;
    
    // test R/W
    uint8_t temp;
    if (hmc5883l_read(hptr, HMC5883L_REG_MODE, 1, &temp))
        return HMC5883L_HANDLE_I2C_READ_FAILED;
    if (hmc5883l_write(hptr, HMC5883L_REG_MODE, 1, &temp))
        return HMC5883L_HANDLE_I2C_WRITE_FAILED;
    
    return HMC5883L_HANDLE_OK;
}

uint8_t hmc5883l_configure(struct hmc5883l_handle* hptr, const struct hmc5883l_configuration new_config)
{
    uint8_t err;

    err = hmc5883l_write(hptr, HMC5883L_REG_CONF_A, 1, &new_config.config_a);
    if (err) return err;
    hptr->config.config_a = new_config.config_a;

    err = hmc5883l_write(hptr, HMC5883L_REG_CONF_B, 1, &new_config.config_b);
    if (err) return err;
    hptr->config.config_b = new_config.config_b;

    err = hmc5883l_write(hptr, HMC5883L_REG_MODE, 1, &new_config.mode);
    if (err) return err;
    hptr->config.mode = new_config.mode;

    return 0;
}

uint8_t hmc5883l_take_single_measurement(struct hmc5883l_handle* hptr)
{
    uint8_t mode = (hptr->config.mode & ~MASK_MODE_MD) | HMC5883L_MD_SINGLE;
    uint8_t err = hmc5883l_write(hptr, HMC5883L_REG_MODE, 1, &mode);
    return err;
}

uint8_t hmc5883l_read_mag(struct hmc5883l_handle* hptr, float* mag_out)
{
    uint8_t buffer[6];
    uint8_t err = hmc5883l_read(hptr, HMC5883L_REG_DATA_BEGIN, 6, (uint8_t*)buffer);
    if (err) return err;

    int16_t x = ((int16_t)buffer[0] << 8) | buffer[1];
    int16_t y = ((int16_t)buffer[4] << 8) | buffer[5];
    int16_t z = ((int16_t)buffer[2] << 8) | buffer[3];

    float resolution = mag_lsb_resolution(hptr->config.config_b);
    mag_out[0] = (x == HMC5883L_OVERFLOW) ? 0 : (x * resolution);
    mag_out[1] = (y == HMC5883L_OVERFLOW) ? 0 : (y * resolution);
    mag_out[2] = (z == HMC5883L_OVERFLOW) ? 0 : (z * resolution);

    return 0;
}

void hmc5883l_convert_mag_from_external(struct hmc5883l_handle* hptr, uint8_t* ext_read, float* mag_out)
{
    int16_t x = ((int16_t)ext_read[0] << 8) | ext_read[1];
    int16_t y = ((int16_t)ext_read[4] << 8) | ext_read[5];
    int16_t z = ((int16_t)ext_read[2] << 8) | ext_read[3];

    float resolution = mag_lsb_resolution(hptr->config.config_b);
    mag_out[0] = (x == HMC5883L_OVERFLOW) ? 0 : (x * resolution);
    mag_out[1] = (y == HMC5883L_OVERFLOW) ? 0 : (y * resolution);
    mag_out[2] = (z == HMC5883L_OVERFLOW) ? 0 : (z * resolution);
}
