#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HMC5883L_DEVICE_ADDRESS     0x1E    // Device Address
#define HMC5883L_REG_DATA_BEGIN     0x03    // Data Register
#define HMC5883L_DATA_LENGTH        6       // Data Length

// number of samples averaged
typedef enum hmc5883l_ma_e
{
    HMC5883L_MA_1           = 0x00,
    HMC5883L_MA_2           = 0x20,
    HMC5883L_MA_4           = 0x40,
    HMC5883L_MA_8           = 0x60
} hmc5883l_ma_t;

// data output rate (sampling rate?)
typedef enum hmc5883l_do_e
{
    HMC5883L_DO_P75HZ       = 0x00,
    HMC5883L_DO_1P5HZ       = 0x04,
    HMC5883L_DO_3HZ         = 0x08,
    HMC5883L_DO_7P5HZ       = 0x0C,
    HMC5883L_DO_15HZ        = 0x10,
    HMC5883L_DO_30HZ        = 0x14,
    HMC5883L_DO_75HZ        = 0x18
} hmc5883l_do_t;

// measurment configuration
typedef enum hmc5883l_ms_e
{
    HMC5883L_MS_NORMAL      = 0x00,
    HMC5883L_MS_POS_BIAS    = 0x01,
    HMC5883L_MS_NEG_BIAS    = 0x02
} hmc5883l_ms_t;

// gain (resolution)
typedef enum hmc5883l_gn_e
{
    HMC5883L_GN_1370        = 0x00, // FSR: ± 1.5 G, Recommend < 0.88 G
    HMC5883L_GN_1090        = 0x20, // FSR: ± 1.8 G, Recommend <  1.3 G
    HMC5883L_GN_820         = 0x40, // FSR: ± 2.5 G, Recommend <  1.9 G
    HMC5883L_GN_660         = 0x60, // FSR: ± 3.1 G, Recommend <  2.5 G
    HMC5883L_GN_440         = 0x80, // FSR: ± 4.6 G, Recommend <  4.0 G
    HMC5883L_GN_390         = 0xA0, // FSR: ± 5.2 G, Recommend <  4.7 G
    HMC5883L_GN_330         = 0xC0, // FSR: ± 6.2 G, Recommend <  5.6 G
    HMC5883L_GN_230         = 0xE0  // FSR: ± 8.9 G, Recommend <  8.1 G
} hmc5883l_gn_t;

// LSB resolution (mG/LSB)
#define HMC5883L_LSB_RESOLUTION(x)      \
(  (x) == HMC5883L_GN_1370  ? 7.299e-1f \
 : (x) == HMC5883L_GN_1090  ? 9.174e-1f \
 : (x) == HMC5883L_GN_820   ? 1.220f    \
 : (x) == HMC5883L_GN_660   ? 1.515f    \
 : (x) == HMC5883L_GN_440   ? 2.273f    \
 : (x) == HMC5883L_GN_390   ? 2.564f    \
 : (x) == HMC5883L_GN_330   ? 3.030f    \
                            : 4.348f)

// high speed i2c (400kHz)
typedef enum hmc5883l_hs_e
{
    HMC5883L_HS_ENABLE      = 0x80,
    HMC5883L_HS_DISABLE     = 0x00
} hmc5883l_hs_t;

// operating mode
typedef enum hmc5883l_md_e
{
    HMC5883L_MD_CONTINUOUS  = 0x00,
    HMC5883L_MD_SINGLE      = 0x01,
    HMC5883L_MD_IDLE        = 0x02
} hmc5883l_md_t;

typedef struct hmc5883l_conf_s
{
    uint8_t config_a;
    uint8_t config_b;
    uint8_t mode;
} __attribute__((aligned(4))) hmc5883l_conf_t;

typedef struct hmc5883l_vec3s_s
{
    int16_t x, y, z;
} __attribute__((aligned(4))) hmc5883l_vec3s_t;

typedef enum hmc5883l_result_e
{
    HMC5883L_OK                 = 0x00,
    HMC5883L_MISSING_I2C_R_CB   = 0x01,
    HMC5883L_MISSING_I2C_W_CB   = 0x02,
    HMC5883L_MISSING_DELAY_CB   = 0x04,
    HMC5883L_I2C_RW_FAIL        = 0x10
} hmc5883l_result_t;

// handle
typedef uint8_t (*hmc5883l_i2c_read_callback)   (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*hmc5883l_i2c_write_callback)  (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*hmc5883l_delay_callback)      (uint32_t milli);

typedef struct hmc5883l_handle_s
{
    hmc5883l_i2c_read_callback i2c_read;
    hmc5883l_i2c_write_callback i2c_write;
    hmc5883l_delay_callback delay;
    hmc5883l_conf_t conf;
    uint8_t i2c_result;
} __attribute__((aligned(4))) hmc5883l_handle_t;

// FIXME: Change those FUCKING retarded function names
// set number of samples averaged
void hmc5883l_set_ma(hmc5883l_conf_t* cptr, hmc5883l_ma_t ma);  
// set data output rate
void hmc5883l_set_do(hmc5883l_conf_t* cptr, hmc5883l_do_t dor);
// set measurement mode
void hmc5883l_set_ms(hmc5883l_conf_t* cptr, hmc5883l_ms_t ms);
// set gain
void hmc5883l_set_gn(hmc5883l_conf_t* cptr, hmc5883l_gn_t gn);
// set high speed i2c
void hmc5883l_set_hs(hmc5883l_conf_t* cptr, hmc5883l_hs_t hs);
// set operating mode
void hmc5883l_set_md(hmc5883l_conf_t* cptr, hmc5883l_md_t md);

void hmc5883l_attach_i2c_read(hmc5883l_handle_t* hptr, hmc5883l_i2c_read_callback callback);
void hmc5883l_attach_i2c_write(hmc5883l_handle_t* hptr, hmc5883l_i2c_write_callback callback);
void hmc5883l_attach_delay(hmc5883l_handle_t* hptr, hmc5883l_delay_callback callback);
hmc5883l_result_t hmc5883l_assert_handle(hmc5883l_handle_t* hptr);

uint8_t hmc5883l_configure(hmc5883l_handle_t* hptr, const hmc5883l_conf_t new_conf);
uint8_t hmc5883l_take_single_measure(hmc5883l_handle_t* hptr);

uint8_t hmc5883l_read_mag(hmc5883l_handle_t* hptr, hmc5883l_vec3s_t* out);
uint8_t hmc5883l_read_mag_from(hmc5883l_handle_t* hptr, hmc5883l_vec3s_t* out, uint64_t from);

float hmc5883l_get_mag_resolution(hmc5883l_handle_t* hptr);

#ifdef __cplusplus
}
#endif