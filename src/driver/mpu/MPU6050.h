#ifndef DKVR_DRIVER_MPU6050
#define DKVR_DRIVER_MPU6050

#include "Driver/IMU.h"
#include "Driver/I2C.h"

// Device Address
#define MPU6050_DEVICE_ADDRESS_A0_LOW   0x68
#define MPU6050_DEVICE_ADDRESS_A0_HIGH  0x69

// Power Management 1
#define MPU_REG_POWER_MGMT_1    0x6B

#define MPU_DEVICE_RESET_BIT    0x80
#define MPU_CLK_INTERNAL_BIT    0x00
#define MPU_CLK_GYRO_X_BIT      0x01
#define MPU_CLK_GYRO_Y_BIT      0x02
#define MPU_CLK_GYRO_Z_BIT      0x03

// Slef Test Register
#define MPU_REG_SELF_TEST_X     0x0D

// Sample Rate Divider
#define MPU_REG_SMPRT_DIV       0x19

// Configuration
#define MPU_REG_CONFIG          0x1A

#define MPU_DLPF_256_HZ_BIT     0x00
#define MPU_DLPF_188_HZ_BIT     0x01
#define MPU_DLPF_98_HZ_BIT      0x02
#define MPU_DLPF_42_HZ_BIT      0x03
#define MPU_DLPF_20_HZ_BIT      0x04
#define MPU_DLPF_10_HZ_BIT      0x05
#define MPU_DLPF_5_HZ_BIT       0x06

// Gyroscope Configuration
#define MPU_REG_GYRO_CONFIG     0x1B

#define MPU_GYRO_SELFTEST_BIT   0xE0
#define MPU_GYRO_FSR_250_BIT    (0x00 << 3)
#define MPU_GYRO_FSR_500_BIT    (0x01 << 3)
#define MPU_GYRO_FSR_1000_BIT   (0x02 << 3)
#define MPU_GYRO_FSR_2000_BIT   (0x03 << 3)

#define MPU_GYRO_FSR_250_RESOLUTION     131.0f
#define MPU_GYRO_FSR_500_RESOLUTION     65.5f
#define MPU_GYRO_FSR_1000_RESOLUTION    32.8f
#define MPU_GYRO_FSR_2000_RESOLUTION    16.4f

// Accelerometer Configuration
#define MPU_REG_ACCEL_CONFIG    0x1C

#define MPU_ACCEL_SELFTEST_BIT  0xE0
#define MPU_ACCEL_FSR_2_BIT     (0x00 << 3)
#define MPU_ACCEL_FSR_4_BIT     (0x01 << 3)
#define MPU_ACCEL_FSR_8_BIT     (0x02 << 3)
#define MPU_ACCEL_FSR_16_BIT    (0x03 << 3)

#define MPU_ACCEL_FSR_2_RESOLUTION      16384.0f
#define MPU_ACCEL_FSR_4_RESOLUTION      8192.0f
#define MPU_ACCEL_FSR_8_RESOLUTION      4096.0f
#define MPU_ACCEL_FSR_16_RESOLUTION     2048.0f

// FIFO Enable
#define MPU_REG_FIFO_EN         0x23

#define MPU_TEMP_FIFO_EN_BIT    0x80
#define MPU_GYRO_X_FIFO_EN_BIT  0x40
#define MPU_GYRO_Y_FIFO_EN_BIT  0x20
#define MPU_GYRO_Z_FIFO_EN_BIT  0x10
#define MPU_ACCEL_FIFO_EN_BIT   0x08
#define MPU_SLV_2_FIFO_EN_BIT   0x04
#define MPU_SLV_1_FIFO_EN_BIT   0x02
#define MPU_SLV_0_FIFO_EN_BIT   0x01

// INT Pin / Bypass Enable Configuration
#define MPU_REG_INT_PIN_CFG     0x37

#define MPU_INT_ACTIVE_L_BIT    0x80
#define MPU_INT_OPEN_DRAIN_BIT  0x40
#define MPU_LATCH_INT_EN_BIT    0x20
#define MPU_INT_RD_CLEAR_BIT    0x10
#define MPU_I2C_BYPASS_EN_BIT   0x02

// Interrupt Enable
#define MPU_REG_INT_ENABLE      0x38

#define MPU_DATA_RDY_EN_BIT     0x01
#define MPU_I2C_MST_INT_EN_BIT  0x08
#define MPU_FIFO_OFLOW_EN_BIT   0x10

// Interrupt Status
#define MPU_REG_INT_STATUS      0x3A

#define MPU_INT_FIFO_OFLOW      0x10
#define MPU_INT_I2C_MST         0x08
#define MPU_INT_DMP             0x02
#define MPU_INT_DATA_RDY        0x01

// User Control
#define MPU_REG_USER_CTRL       0x6A

#define MPU_DMP_EN_BIT          0x80
#define MPU_FIFO_EN_BIT         0x40
#define MPU_I2C_MST_EN_BIT      0x20
#define MPU_DMP_RESET_BIT       0x08
#define MPU_FIFO_RESET_BIT      0x04
#define MPU_I2C_MST_RESET_BIT   0x02
#define MPU_SIG_COND_RESET_BIT  0x01

// I2C Master Control
#define MPU_REG_I2C_MST_CTRL    0x24

#define MPU_WAIT_FOR_ES_BIT     0x40
#define MPU_I2C_MST_CLK_400_BIT 0x0D

// I2C Slave 0 Control
#define MPU_REG_I2C_SLV0_ADDR   0x25
#define MPU_REG_I2C_SLV0_REG    0x26
#define MPU_REG_I2C_SLV0_CTRL   0x27

#define MPU_I2C_SLV0_R_MODE_BIT 0x80
#define MPU_I2C_SLV0_EN_BIT     0x80

// External Sensor Data
#define MPU_REG_EXT_SENS_DATA   0x49
#define MPU_REG_I2C_SLV0_DO     0x63


// Raw Data Out Register
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_ACCEL_XOUT_L    0x3C
#define MPU_REG_ACCEL_YOUT_H    0x3D
#define MPU_REG_ACCEL_YOUT_L    0x3E
#define MPU_REG_ACCEL_ZOUT_H    0x3F
#define MPU_REG_ACCEL_ZOUT_L    0x40

#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_TEMP_OUT_L      0x42

#define MPU_REG_GYRO_XOUT_H     0x43
#define MPU_REG_GYRO_XOUT_L     0x44
#define MPU_REG_GYRO_YOUT_H     0x45
#define MPU_REG_GYRO_YOUT_L     0x46
#define MPU_REG_GYRO_ZOUT_H     0x47
#define MPU_REG_GYRO_ZOUT_L     0x48


namespace DKVR
{
    namespace Driver
    {
        struct MPU6050Configurations {
            uint8_t clksel;     // Clock Source (Register 107 - Power Managment 1)
            uint8_t smprtDiv;   // Sample Rate Divider (Register 25 - Sample Rate Divider)
            uint8_t dlpf;       // Digital Low Pass Filter (Register 26 - Configuration)
            uint8_t gyroFsr;    // Gyroscope Full Scale Range (Register 27 - Gyroscope Configuration)
            uint8_t accelFsr;   // Accelerometer Full Scale Range (Register 28 - Accelerometer Configuration)
            uint8_t fifoEn;     // FIFO Enable Bit Mask (Register 35 - FIFO Enable)
            uint8_t intConfig;  // INT Pin Config (Register 55 - INT Pin / Bypass Enable Configuration)
            uint8_t intEnable;  // Interrupt Enable (Register 56 - Interrupt Enable)
        };

        class MPU6050 : public Gyroscope, public Accelerometer, public Interruptible
        {
        public:
            MPU6050(bool a0, const MPU6050Configurations &arg);
            uint8_t InitializeGyro() override;
            uint8_t InitializeAccel() override;
            uint8_t ReadRawGyro(DataType::Vector3s &gyro) override;
            uint8_t ReadRawAccel(DataType::Vector3s &accel) override;
            uint8_t ReadInterrupt(uint8_t &intout) override;
            void HandleInterrupt(uint8_t interrupt) override;
            bool GetDataReadyInterrupt() override { return dataReady; };

            const uint8_t deviceAddress;

        private:
            uint8_t RunSelfTest();

            MPU6050Configurations configuration;
            uint8_t sharedBuffer[8];
            bool dataReady = false;
        };
    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_MPU6050
