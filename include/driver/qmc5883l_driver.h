#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_DEVICE_ADDRESS     0x0D    // Device Address
#define QMC5883L_REG_DATA_BEGIN     0x00    // Data Register
#define QMC5883L_DATA_LENGTH        6       // Data Length

// operational mode
typedef enum qmc5883l_mode_e
{
    QMC5883L_MODE_STANDBY       = 0x00,
    QMC5883L_MODE_CONTINUOUS    = 0x01
} qmc5883l_mode_t;

// output data update rate
typedef enum qmc5883l_odr_e
{
    QMC5883L_ODR_10HZ           = 0x00,
    QMC5883L_ODR_50HZ           = 0x04,
    QMC5883L_ODR_100HZ          = 0x08,
    QMC5883L_ODR_200HZ          = 0x0C,
} qmc5883l_odr_t;

// full scale range
typedef enum qmc5883l_rng_e
{
    QMC5883L_RNG_2G             = 0x00,
    QMC5883L_RNG_8G             = 0x10
} qmc5883l_rng_t;

// LSB resolution (mG/LSB)
#define QMC5883L_LSB_RESOLUTION(x)  ((x) == QMC5883L_RNG_2G ? 6.104e-2f : 2.441e-1f)

// over sample rate
typedef enum qmc5883l_osr_e
{
    QMC5883L_OSR_512            = 0x00,
    QMC5883L_OSR_256            = 0x40,
    QMC5883L_OSR_128            = 0x80,
    QMC5883L_OSR_64             = 0xC0
} qmc5883l_osr_t;

// interrupt enable
typedef enum qmc5883l_int_en_e
{
    QMC5883L_INT_ENABLE         = 0x00,
    QMC5883L_INT_DISABLE        = 0x01
} qmc5883l_int_en_t;

// pointer roll-over enable
typedef enum qmc5883l_rol_pnt_e
{
    QMC5883L_ROL_PNT_NORMAL     = 0x00,
    QMC5883L_ROL_PNT_ENABLE     = 0x40
} qmc5883l_rol_pnt_t;


typedef struct qmc5883l_conf_s
{
    uint8_t control1;
    uint8_t control2;
} qmc5883l_conf_t;

typedef enum qmc5883l_result_e
{
    QMC5883L_OK                 = 0x00,
    QMC5883L_MISSING_I2C_R_CB   = 0x01,
    QMC5883L_MISSING_I2C_W_CB   = 0x02,
    QMC5883L_MISSING_DELAY_CB   = 0x04,
    QMC5883L_MISSING_MASTER_CB  = 0x08,
    QMC5883L_I2C_RW_FAIL        = 0x10
} qmc5883l_result_t;

typedef struct qmc5883l_vec3s_s
{
    int16_t x, y, z;
} __attribute__((aligned(4))) qmc5883l_vec3s_t;

// handle
typedef uint8_t (*qmc5883l_i2c_read_callback)   (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*qmc5883l_i2c_write_callback)  (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*qmc5883l_delay_callback)      (uint32_t milli);

typedef struct qmc5883l_handle_s
{
    qmc5883l_i2c_read_callback i2c_read;
    qmc5883l_i2c_write_callback i2c_write;
    qmc5883l_delay_callback delay;
    qmc5883l_conf_t conf;
    uint8_t i2c_result;
} __attribute__((aligned(4))) qmc5883l_handle_t;

void qmc5883l_set_mode(qmc5883l_conf_t* cptr, qmc5883l_mode_t mode);
void qmc5883l_set_odr(qmc5883l_conf_t* cptr, qmc5883l_odr_t odr);
void qmc5883l_set_rng(qmc5883l_conf_t* cptr, qmc5883l_rng_t rng);
void qmc5883l_set_osr(qmc5883l_conf_t* cptr, qmc5883l_osr_t osr);
void qmc5883l_set_int_en(qmc5883l_conf_t* cptr, qmc5883l_int_en_t int_en);
void qmc5883l_set_rol_pnt(qmc5883l_conf_t* cptr, qmc5883l_rol_pnt_t rol_pnt);

void qmc5883l_attach_i2c_read(qmc5883l_handle_t* hptr, qmc5883l_i2c_read_callback callback);
void qmc5883l_attach_i2c_write(qmc5883l_handle_t* hptr, qmc5883l_i2c_write_callback callback);
void qmc5883l_attach_delay(qmc5883l_handle_t* hptr, qmc5883l_delay_callback callback);
qmc5883l_result_t qmc5883l_assert_handle(qmc5883l_handle_t* hptr);

uint8_t qmc5883l_reset(qmc5883l_handle_t* hptr);
uint8_t qmc5883l_configure(qmc5883l_handle_t* hptr, const qmc5883l_conf_t new_conf);

uint8_t qmc5883l_read_mag(qmc5883l_handle_t* hptr, qmc5883l_vec3s_t* out);
uint8_t qmc5883l_read_mag_from(qmc5883l_handle_t* hptr, qmc5883l_vec3s_t* out, uint64_t from);

float qmc5883l_get_mag_resolution(qmc5883l_handle_t* hptr);

#ifdef __cplusplus
}
#endif