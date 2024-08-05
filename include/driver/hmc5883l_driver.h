#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HMC5883L_DEVICE_ADDRESS     0x1E    // Device Address
#define HMC5883L_REG_DATA_BEGIN     0x03    // Data Register
#define HMC5883L_DATA_LENGTH        6       // Data Length

// number of samples averaged
typedef enum
{
    HMC5883L_MA_1           = 0x00,
    HMC5883L_MA_2           = 0x20,
    HMC5883L_MA_4           = 0x40,
    HMC5883L_MA_8           = 0x60
} hmc5883l_ma;

// data output rate (sampling rate?)
typedef enum
{
    HMC5883L_DO_P75HZ       = 0x00,
    HMC5883L_DO_1P5HZ       = 0x04,
    HMC5883L_DO_3HZ         = 0x08,
    HMC5883L_DO_7P5HZ       = 0x0C,
    HMC5883L_DO_15HZ        = 0x10,
    HMC5883L_DO_30HZ        = 0x14,
    HMC5883L_DO_75HZ        = 0x18
} hmc5883l_do;

// measurment configuration
typedef enum
{
    HMC5883L_MS_NORMAL      = 0x00,
    HMC5883L_MS_POS_BIAS    = 0x01,
    HMC5883L_MS_NEG_BIAS    = 0x02
} hmc5883l_ms;

// gain (resolution)
typedef enum
{
    HMC5883L_GN_1370        = 0x00, // FSR: ± 1.5 G, Recommend < 0.88 G
    HMC5883L_GN_1090        = 0x20, // FSR: ± 1.8 G, Recommend <  1.3 G
    HMC5883L_GN_820         = 0x40, // FSR: ± 2.5 G, Recommend <  1.9 G
    HMC5883L_GN_660         = 0x60, // FSR: ± 3.1 G, Recommend <  2.5 G
    HMC5883L_GN_440         = 0x80, // FSR: ± 4.6 G, Recommend <  4.0 G
    HMC5883L_GN_390         = 0xA0, // FSR: ± 5.2 G, Recommend <  4.7 G
    HMC5883L_GN_330         = 0xC0, // FSR: ± 6.2 G, Recommend <  5.6 G
    HMC5883L_GN_230         = 0xE0  // FSR: ± 8.9 G, Recommend <  8.1 G
} hmc5883l_gn;

// operating mode
typedef enum
{
    HMC5883L_MD_CONTINUOUS  = 0x00,
    HMC5883L_MD_SINGLE      = 0x01,
    HMC5883L_MD_IDLE        = 0x02
} hmc5883l_md;

// configuration struct
struct hmc5883l_configuration
{
    uint8_t config_a;
    uint8_t config_b;
    uint8_t mode;
} __attribute__((aligned(4)));

// handle test result
typedef enum
{
    HMC5883L_HANDLE_OK               = 0x00,
    HMC5883L_HANDLE_MISSING_CALLBACK = 0x01,
    HMC5883L_HANDLE_I2C_READ_FAILED  = 0x02,
    HMC5883L_HANDLE_I2C_WRITE_FAILED = 0x03
} hmc5883l_handle_test_result;

// callback
typedef uint8_t (*hmc5883l_i2c_read_callback)   (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*hmc5883l_i2c_write_callback)  (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*hmc5883l_delay_callback)      (uint32_t milli);

// handle struct
struct hmc5883l_handle
{
    hmc5883l_i2c_read_callback i2c_read;
    hmc5883l_i2c_write_callback i2c_write;
    hmc5883l_delay_callback delay;
    struct hmc5883l_configuration config;
} __attribute__((aligned(4)));


void hmc5883l_set_measurement_averaged(struct hmc5883l_configuration* config, hmc5883l_ma ma);
void hmc5883l_set_data_output_rate(struct hmc5883l_configuration* config, hmc5883l_do dor);
void hmc5883l_set_measurement_configuration(struct hmc5883l_configuration* config, hmc5883l_ms ms);
void hmc5883l_set_gain(struct hmc5883l_configuration* config, hmc5883l_gn gain);
void hmc5883l_set_high_speed_i2c(struct hmc5883l_configuration* config, int enable);
void hmc5883l_set_operating_mode(struct hmc5883l_configuration* config, hmc5883l_md mode);

hmc5883l_handle_test_result hmc5883l_test_handle(struct hmc5883l_handle* hptr);

uint8_t hmc5883l_configure(struct hmc5883l_handle* hptr, const struct hmc5883l_configuration new_config);
uint8_t hmc5883l_take_single_measurement(struct hmc5883l_handle* hptr);

uint8_t hmc5883l_read_mag(struct hmc5883l_handle* hptr, float* mag_out);
void hmc5883l_convert_mag_from_external(struct hmc5883l_handle* hptr, uint8_t* ext_read, float* mag_out);

#ifdef __cplusplus
}
#endif