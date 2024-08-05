#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_DEVICE_ADDRESS     0x0D    // Device Address
#define QMC5883L_REG_DATA_BEGIN     0x00    // Data Register
#define QMC5883L_DATA_LENGTH        6       // Data Length

// operational mode
typedef enum
{
    QMC5883L_MODE_STANDBY       = 0x00,
    QMC5883L_MODE_CONTINUOUS    = 0x01
} qmc5883l_mode;

// output data update rate
typedef enum
{
    QMC5883L_ODR_10HZ           = 0x00,
    QMC5883L_ODR_50HZ           = 0x04,
    QMC5883L_ODR_100HZ          = 0x08,
    QMC5883L_ODR_200HZ          = 0x0C,
} qmc5883l_odr;

// full scale range
typedef enum
{
    QMC5883L_RNG_2G             = 0x00,
    QMC5883L_RNG_8G             = 0x10
} qmc5883l_rng;

// over sample rate
typedef enum
{
    QMC5883L_OSR_512            = 0x00,
    QMC5883L_OSR_256            = 0x40,
    QMC5883L_OSR_128            = 0x80,
    QMC5883L_OSR_64             = 0xC0
} qmc5883l_osr;

// configuration struct
struct qmc5883l_configuration
{
    uint8_t control1, control2;
};

// handle test result
typedef enum 
{
    QMC5883L_HANDLE_OK               = 0x00,
    QMC5883L_HANDLE_MISSING_CALLBACK = 0x01,
    QMC5883L_HANDLE_I2C_READ_FAILED  = 0x02,
    QMC5883L_HANDLE_I2C_WRITE_FAILED = 0x03
} qmc5883l_handle_test_result;


// callback
typedef uint8_t (*qmc5883l_i2c_read_callback)   (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*qmc5883l_i2c_write_callback)  (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*qmc5883l_delay_callback)      (uint32_t milli);

// handle struct
struct qmc5883l_handle
{
    qmc5883l_i2c_read_callback i2c_read;
    qmc5883l_i2c_write_callback i2c_write;
    qmc5883l_delay_callback delay;
    struct qmc5883l_configuration config;
} __attribute__((aligned(4)));


void qmc5883l_set_mode(struct qmc5883l_configuration* config, qmc5883l_mode mode);
void qmc5883l_set_output_data_rate(struct qmc5883l_configuration* config, qmc5883l_odr odr);
void qmc5883l_set_scale_range(struct qmc5883l_configuration* config, qmc5883l_rng rng);
void qmc5883l_set_over_sample_rate(struct qmc5883l_configuration* config, qmc5883l_osr osr);
void qmc5883l_set_interrupt(struct qmc5883l_configuration* config, int enable);
void qmc5883l_set_pointer_rollover(struct qmc5883l_configuration* config, int enable);

qmc5883l_handle_test_result qmc5883l_test_handle(struct qmc5883l_handle* hptr);

uint8_t qmc5883l_reset(struct qmc5883l_handle* hptr);
uint8_t qmc5883l_configure(struct qmc5883l_handle* hptr, const struct qmc5883l_configuration new_conf);
uint8_t qmc5883l_read_mag(struct qmc5883l_handle* hptr, float* mag_out);
void qmc5883l_convert_mag_from_external(struct qmc5883l_handle* hptr, uint8_t* ext_read, float* mag_out);

#ifdef __cplusplus
}
#endif