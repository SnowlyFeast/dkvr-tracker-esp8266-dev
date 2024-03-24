#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Device Address
typedef enum mpu6050_address_e
{
    MPU6050_DEVICE_ADDRESS_A0_LOW   = 0x68,
    MPU6050_DEVICE_ADDRESS_A0_HIGH  = 0x69
} mpu6050_address_t;

// clock source configuration
typedef enum mpu6050_clksel_e
{
    MPU6050_CLK_INTERNAL_BIT        = 0x00,
    MPU6050_CLK_GYRO_X_BIT          = 0x01,
    MPU6050_CLK_GYRO_Y_BIT          = 0x02,
    MPU6050_CLK_GYRO_Z_BIT          = 0x03
} mpu6050_clksel_t;

// digital LPF configuration
typedef enum mpu6050_dlpf_e
{
    MPU6050_DLPF_256_HZ_BIT         = 0x00,
    MPU6050_DLPF_188_HZ_BIT         = 0x01,
    MPU6050_DLPF_98_HZ_BIT          = 0x02,
    MPU6050_DLPF_42_HZ_BIT          = 0x03,
    MPU6050_DLPF_20_HZ_BIT          = 0x04,
    MPU6050_DLPF_10_HZ_BIT          = 0x05,
    MPU6050_DLPF_5_HZ_BIT           = 0x06
} mpu6050_dlpf_t;

// gyroscope configuration
typedef enum mpu6050_gyro_fsr_e
{
    MPU6050_GYRO_FSR_250_BIT        = (0x00 << 3),
    MPU6050_GYRO_FSR_500_BIT        = (0x01 << 3),
    MPU6050_GYRO_FSR_1000_BIT       = (0x02 << 3),
    MPU6050_GYRO_FSR_2000_BIT       = (0x03 << 3)
} mpu6050_gyro_fsr_t;

// gyroscope LSB resolution (°/s/LSB)
#define MPU6050_GYRO_LSB_RESOLUTION(x)              \
(  x == MPU6050_GYRO_FSR_250_BIT    ? 7.629e-3f     \
 : x == MPU6050_GYRO_FSR_500_BIT    ? 1.526e-2f     \
 : x == MPU6050_GYRO_FSR_1000_BIT   ? 3.052e-2f     \
                                    : 6.104e-2f)

// accelerometer configuration
typedef enum mpu6050_accel_fsr_e
{
    MPU6050_ACCEL_FSR_2_BIT         = (0x00 << 3),
    MPU6050_ACCEL_FSR_4_BIT         = (0x01 << 3),
    MPU6050_ACCEL_FSR_8_BIT         = (0x02 << 3),
    MPU6050_ACCEL_FSR_16_BIT        = (0x03 << 3)
} mpu6050_accel_fsr_t;

// accelerometer LSB resolution (g/LSB)
#define MPU6050_ACCEL_LSB_RESOLUTION(x)             \
  (x == MPU6050_ACCEL_FSR_2_BIT     ? 6.104e-5f     \
 : x == MPU6050_ACCEL_FSR_4_BIT     ? 1.221e-4f     \
 : x == MPU6050_ACCEL_FSR_8_BIT     ? 2.441e-4f     \
                                    : 4.883e-4f)

// FIFO enable
typedef uint8_t mpu6050_fifo_en_t;
#define MPU6050_ENABLE_TEMP_FIFO(mpu6050_fifo_en_t)     (mpu6050_fifo_en_t |= 0x80)
#define MPU6050_ENABLE_GYRO_X_FIFO(mpu6050_fifo_en_t)   (mpu6050_fifo_en_t |= 0x40)
#define MPU6050_ENABLE_GYRO_Y_FIFO(mpu6050_fifo_en_t)   (mpu6050_fifo_en_t |= 0x20)
#define MPU6050_ENABLE_GYRO_Z_FIFO(mpu6050_fifo_en_t)   (mpu6050_fifo_en_t |= 0x10)
#define MPU6050_ENABLE_ACCEL_FIFO(mpu6050_fifo_en_t)    (mpu6050_fifo_en_t |= 0x08)
#define MPU6050_ENABLE_SLV_2_FIFO(mpu6050_fifo_en_t)    (mpu6050_fifo_en_t |= 0x04)
#define MPU6050_ENABLE_SLV_1_FIFO(mpu6050_fifo_en_t)    (mpu6050_fifo_en_t |= 0x02)
#define MPU6050_ENABLE_SLV_0_FIFO(mpu6050_fifo_en_t)    (mpu6050_fifo_en_t |= 0x01)

// INT Pin / Bypass enable configuration
typedef uint8_t mpu6050_int_conf_t;
#define MPU6050_ENABLE_INT_ACTIVE_LOW(mpu6050_int_conf_t)   (mpu6050_int_conf_t |= 0x80)
#define MPU6050_ENABLE_INT_OPEN_DRAIN(mpu6050_int_conf_t)   (mpu6050_int_conf_t |= 0x40)
#define MPU6050_ENABLE_LATCH_INT_EN(mpu6050_int_conf_t)     (mpu6050_int_conf_t |= 0x20)
#define MPU6050_ENABLE_INT_RD_CLR(mpu6050_int_conf_t)       (mpu6050_int_conf_t |= 0x10)
#define MPU6050_ENABLE_I2C_BYPASS_EN(mpu6050_int_conf_t)    (mpu6050_int_conf_t |= 0x02)

// interrupt enable
typedef uint8_t mpu6050_int_en_t;
#define MPU6050_ENABLE_FIFO_OFLOW_EN(mpu6050_int_en_t)      (mpu6050_int_en_t |= 0x10)
#define MPU6050_ENABLE_I2C_MST_INT_EN(mpu6050_int_en_t)     (mpu6050_int_en_t |= 0x08)
#define MPU6050_ENABLE_DATA_RDY_EN(mpu6050_int_en_t)        (mpu6050_int_en_t |= 0x01)

// interrupt status 
typedef uint8_t mpu6050_interrupt_t;
#define MPU6050_IS_INT_FIFO_OFLOW(mpu6050_interrupt_t)      (mpu6050_interrupt_t & 0x10)
#define MPU6050_IS_INT_I2C_MST(mpu6050_interrupt_t)         (mpu6050_interrupt_t & 0x08)
#define MPU6050_IS_INT_DATA_RDY(mpu6050_interrupt_t)        (mpu6050_interrupt_t & 0x01)

// user control
typedef uint8_t mpu6050_user_ctrl_t;
#define MPU6050_ENABLE_FIFO_EN(mpu6050_user_ctrl_t)         (mpu6050_user_ctrl_t |= 0x40)
#define MPU6050_ENABLE_I2C_MST_EN(mpu6050_user_ctrl_t)      (mpu6050_user_ctrl_t |= 0x20)
#define MPU6050_ENABLE_I2C_IF_DIS(mpu6050_user_ctrl_t)      (mpu6050_user_ctrl_t |= 0x10)
#define MPU6050_ENABLE_FIFO_RESET(mpu6050_user_ctrl_t)      (mpu6050_user_ctrl_t |= 0x04)
#define MPU6050_ENABLE_I2C_MST_RESET(mpu6050_user_ctrl_t)   (mpu6050_user_ctrl_t |= 0x02)
#define MPU6050_ENABLE_SIG_COND_RESET(mpu6050_user_ctrl_t)  (mpu6050_user_ctrl_t |= 0x01)

// configuration struct
typedef struct mpu6050_conf_s
{
    uint8_t address;
    uint8_t smplrt;         // Actual Sampling Rate             (required to calculate smplrt_div)
    uint8_t clksel;         // Clock Source                     (Register 107 - Power Managment 1)
    uint8_t smplrt_div;     // Sample Rate Divider              (Register  25 - Sample Rate Divider)
    uint8_t dlpf;           // Digital Low Pass Filter          (Register  26 - Configuration)
    uint8_t gyro_fsr;       // Gyroscope Full Scale Range       (Register  27 - Gyroscope Configuration)
    uint8_t accel_fsr;      // Accelerometer Full Scale Range   (Register  28 - Accelerometer Configuration)
    uint8_t fifo_en;        // FIFO Enable Bit Mask             (Register  35 - FIFO Enable)
    uint8_t int_conf;       // INT Pin Config                   (Register  55 - INT Pin / Bypass Enable Configuration)
    uint8_t int_enable;     // Interrupt Enable                 (Register  56 - Interrupt Enable)
    uint8_t user_ctrl;      // User Control                     (Register 106 - User Control)
} __attribute__((aligned(4))) mpu6050_conf_t;

typedef struct mpu6050_external_s
{
    uint8_t addr;
    uint8_t reg;
    uint8_t len;
} mpu6050_external_t;

// handle struct
typedef uint8_t (*mpu6050_i2c_read_callback)    (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*mpu6050_i2c_write_callback)   (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*mpu6050_delay_callback)       (uint32_t milli);

typedef struct mpu6050_handle_s
{
    mpu6050_i2c_read_callback i2c_read;
    mpu6050_i2c_write_callback i2c_write;
    mpu6050_delay_callback delay;
    mpu6050_external_t external;
    mpu6050_conf_t conf;
    uint8_t i2c_result;
} __attribute__((aligned(4))) mpu6050_handle_t;

// return type
typedef enum mpu6050_result_e
{
    MPU6050_OK                  = 0x00,
    MPU6050_MISSING_I2C_R_CB    = 0x01,
    MPU6050_MISSING_I2C_W_CB    = 0x02,
    MPU6050_MISSING_DELAY_CB    = 0x04,
    MPU6050_I2C_RW_FAIL         = 0x08,
    MPU6050_SELF_TEST_FAIL_G    = 0x10,
    MPU6050_SELF_TEST_FAIL_A    = 0x20
} mpu6050_result_t;

typedef struct mpu6050_vec3s_s
{
    int16_t x, y, z;
} __attribute__ ((aligned(4))) mpu6050_vec3s_t;

void mpu6050_set_clksel(mpu6050_conf_t* cptr, mpu6050_clksel_t clksel);
void mpu6050_set_sampling_rate(mpu6050_conf_t* cptr, uint8_t smplrt);
void mpu6050_set_dlpf(mpu6050_conf_t* cptr, mpu6050_dlpf_t dlpf);
void mpu6050_set_gyro_fsr(mpu6050_conf_t* cptr, mpu6050_gyro_fsr_t gyro_fsr);
void mpu6050_set_accel_fsr(mpu6050_conf_t* cptr, mpu6050_accel_fsr_t accel_fsr);
void mpu6050_set_fifo_en(mpu6050_conf_t* cptr, mpu6050_fifo_en_t fifo_en);
void mpu6050_set_int_conf(mpu6050_conf_t* cptr, mpu6050_int_conf_t int_conf);
void mpu6050_set_int_en(mpu6050_conf_t* cptr, mpu6050_int_en_t int_en);

void mpu6050_attach_i2c_read(mpu6050_handle_t* hptr, mpu6050_i2c_read_callback callback);
void mpu6050_attach_i2c_write(mpu6050_handle_t* hptr, mpu6050_i2c_write_callback callback);
void mpu6050_attach_delay(mpu6050_handle_t* hptr, mpu6050_delay_callback callback);
void mpu6050_set_address(mpu6050_handle_t* hptr, mpu6050_address_t address);
mpu6050_result_t mpu6050_assert_handle(mpu6050_handle_t* hptr);

uint8_t mpu6050_reset(mpu6050_handle_t* hptr);
uint8_t mpu6050_configure(mpu6050_handle_t* hptr, mpu6050_conf_t* cptr);
mpu6050_result_t mpu6050_run_self_test(mpu6050_handle_t* hptr);
uint8_t mpu6050_enable_bypass(mpu6050_handle_t* hptr);
uint8_t mpu6050_disable_bypass(mpu6050_handle_t* hptr);
uint8_t mpu6050_setup_external(mpu6050_handle_t* hptr, mpu6050_external_t external);

uint8_t mpu6050_read_intterrupt(mpu6050_handle_t* hptr, mpu6050_interrupt_t* out);
uint8_t mpu6050_read_gyro(mpu6050_handle_t* hptr, mpu6050_vec3s_t* out);
uint8_t mpu6050_read_accel(mpu6050_handle_t* hptr, mpu6050_vec3s_t* out);
uint8_t mpu6050_read_external(mpu6050_handle_t* hptr, uint8_t* out);

float mpu6050_get_gyro_resolution(mpu6050_handle_t* hptr);
float mpu6050_get_accel_resolution(mpu6050_handle_t* hptr);

#ifdef __cplusplus
}
#endif