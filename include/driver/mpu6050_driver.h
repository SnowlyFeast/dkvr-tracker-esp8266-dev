#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// clock source configuration
typedef enum
{
    MPU6050_CLK_INTERNAL_BIT        = 0x00,
    MPU6050_CLK_GYRO_X_BIT          = 0x01,
    MPU6050_CLK_GYRO_Y_BIT          = 0x02,
    MPU6050_CLK_GYRO_Z_BIT          = 0x03
} mpu6050_clksel;

// digital LPF configuration
typedef enum
{
    MPU6050_DLPF_256_HZ_BIT         = 0x00,
    MPU6050_DLPF_188_HZ_BIT         = 0x01,
    MPU6050_DLPF_98_HZ_BIT          = 0x02,
    MPU6050_DLPF_42_HZ_BIT          = 0x03,
    MPU6050_DLPF_20_HZ_BIT          = 0x04,
    MPU6050_DLPF_10_HZ_BIT          = 0x05,
    MPU6050_DLPF_5_HZ_BIT           = 0x06
} mpu6050_dlpf;

// gyroscope configuration
typedef enum
{
    MPU6050_GYRO_FSR_250_BIT        = (0x00 << 3),
    MPU6050_GYRO_FSR_500_BIT        = (0x01 << 3),
    MPU6050_GYRO_FSR_1000_BIT       = (0x02 << 3),
    MPU6050_GYRO_FSR_2000_BIT       = (0x03 << 3)
} mpu6050_gyro_fsr;

// accelerometer configuration
typedef enum
{
    MPU6050_ACCEL_FSR_2_BIT         = (0x00 << 3),
    MPU6050_ACCEL_FSR_4_BIT         = (0x01 << 3),
    MPU6050_ACCEL_FSR_8_BIT         = (0x02 << 3),
    MPU6050_ACCEL_FSR_16_BIT        = (0x03 << 3)
} mpu6050_accel_fsr;

// handle test result
typedef enum
{
    MPU6050_HANDLE_OK               = 0x00,
    MPU6050_HANDLE_MISSING_CALLBACK = 0x01,
    MPU6050_HANDLE_I2C_READ_FAILED  = 0x02,
    MPU6050_HANDLE_I2C_WRITE_FAILED = 0x03
} mpu6050_handle_test_result;

// self test result
typedef enum
{
    MPU6050_SELF_TEST_PASSED        = 0x00,
    MPU6050_SELF_TEST_GYRO_FAILED   = 0x01,
    MPU6050_SELF_TEST_ACCEL_FAILED  = 0x02,
    MPU6050_SELF_TEST_I2C_ERROR     = 0x04
} mpu6050_self_test_result;

// interrupt code
typedef uint8_t mpu6050_interrupt;

// configuration struct
struct mpu6050_configuration
{
    uint8_t ad0;
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

    uint8_t ext_addr;       // Address of Slave0                (Register  37 - I2C Slave 0 Control)
    uint8_t ext_reg;        // Register address of Slave0       (Register  38 - I2C Slave 0 Control)
    uint8_t ext_len;        // Number of bytes to read          (Register  39 - I2C Slave 0 Control)
} __attribute__((aligned(4)));

// callback
typedef uint8_t (*mpu6050_i2c_read_callback)    (uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buffer, uint32_t timeout);
typedef uint8_t (*mpu6050_i2c_write_callback)   (uint8_t addr, uint8_t reg, uint8_t len, const uint8_t* buffer, uint32_t timeout);
typedef void    (*mpu6050_delay_callback)       (uint32_t milli);

// handle
struct mpu6050_handle
{
    mpu6050_i2c_read_callback i2c_read;
    mpu6050_i2c_write_callback i2c_write;
    mpu6050_delay_callback delay;
    struct mpu6050_configuration config;
} __attribute__((aligned(4)));


/* -------------------------------------------------------------------------- */
/*                           configuration functions                          */
/* -------------------------------------------------------------------------- */
// i2c address setup
void mpu6050_set_ad0(struct mpu6050_handle* hptr, int high);

// main configuration
void mpu6050_set_clksel(struct mpu6050_configuration* config, mpu6050_clksel clksel);
void mpu6050_set_sampling_rate(struct mpu6050_configuration* config, uint8_t sampling_rate);
void mpu6050_set_dlpf(struct mpu6050_configuration* config, mpu6050_dlpf dlpf);
void mpu6050_set_gyro_fsr(struct mpu6050_configuration* config, mpu6050_gyro_fsr gyro_fsr);
void mpu6050_set_accel_fsr(struct mpu6050_configuration* config, mpu6050_accel_fsr accel_fsr);

// FIFO enable
void mpu6050_enable_temp_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_gyro_x_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_gyro_y_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_gyro_z_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_accel_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_slv_2_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_slv_1_fifo(struct mpu6050_configuration* config);
void mpu6050_enable_slv_0_fifo(struct mpu6050_configuration* config);

// INT Pin / Bypass enable configuration
void mpu6050_enable_int_active_low(struct mpu6050_configuration* config);
void mpu6050_enable_int_open_drain(struct mpu6050_configuration* config);
void mpu6050_enable_latch_int_en(struct mpu6050_configuration* config);
void mpu6050_enable_int_rd_clr(struct mpu6050_configuration* config);
void mpu6050_enable_i2c_bypass_en(struct mpu6050_configuration* config);

// interrupt enable
void mpu6050_enable_fifo_oflow_en(struct mpu6050_configuration* config);
void mpu6050_enable_i2c_mst_int_en(struct mpu6050_configuration* config);
void mpu6050_enable_data_rdy_en(struct mpu6050_configuration* config);

// user control
void mpu6050_enable_fifo_en(struct mpu6050_configuration* config);
void mpu6050_enable_i2c_mst_en(struct mpu6050_configuration* config);
void mpu6050_enable_i2c_if_dis(struct mpu6050_configuration* config);
void mpu6050_enable_fifo_reset(struct mpu6050_configuration* config);
void mpu6050_enable_i2c_mst_reset(struct mpu6050_configuration* config);
void mpu6050_enable_sig_cond_reset(struct mpu6050_configuration* config);

// interrupt check
int mpu6050_is_fifo_oflow_interrupt(mpu6050_interrupt interrupt);
int mpu6050_is_i2c_mst_interrupt(mpu6050_interrupt interrupt);
int mpu6050_is_data_rdy_interrupt(mpu6050_interrupt interrupt);

/* -------------------------------------------------------------------------- */
/*                               main functions                               */
/* -------------------------------------------------------------------------- */
mpu6050_handle_test_result mpu6050_test_handle(struct mpu6050_handle* hptr);
mpu6050_self_test_result mpu6050_run_self_test(struct mpu6050_handle* hptr);

uint8_t mpu6050_reset(struct mpu6050_handle* hptr);
uint8_t mpu6050_configure(struct mpu6050_handle* hptr, struct mpu6050_configuration* new_config);
uint8_t mpu6050_enable_bypass(struct mpu6050_handle* hptr);
uint8_t mpu6050_disable_bypass(struct mpu6050_handle* hptr);
uint8_t mpu6050_enable_external(struct mpu6050_handle* hptr);

uint8_t mpu6050_read_interrupt(struct mpu6050_handle* hptr, mpu6050_interrupt* int_out);
uint8_t mpu6050_read_gyro(struct mpu6050_handle* hptr, float* gyr_out);
uint8_t mpu6050_read_accel(struct mpu6050_handle* hptr, float* acc_out);
uint8_t mpu6050_read_external(struct mpu6050_handle* hptr, uint8_t* ext_out);


#ifdef __cplusplus
}
#endif