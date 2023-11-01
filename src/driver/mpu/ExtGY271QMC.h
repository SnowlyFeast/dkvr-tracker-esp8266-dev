#ifndef DKVR_DRIVER_EXTGY271QMC
#define DKVR_DRIVER_EXTGY271QMC

// TODO: ifndef guard QMC vs HMC

#include "Driver/IMU.h"
#include "Driver/I2C.h"
#include "MPU6050.h"

// GY271(QMC5883L) Address and Data Reg
#define GY271_DEVICE_ADDRESS    0x0D
#define GY271_REG_DATA_BEGIN    0x00
#define GY271_DATA_SIZE         6

// GY271(QMC5883L) Control Register 1
#define GY271_REG_CONTROL_1     0x09

#define GY271_MODE_STANDBY      0x00
#define GY271_MODE_CONTINUOUS   0x01
#define GY271_ODR_10HZ          0x00
#define GY271_ODR_50HZ          0x04
#define GY271_ODR_100HZ         0x08
#define GY271_ODR_200HZ         0x0C
#define GY271_FSR_2G            0x00
#define GY271_FSR_8G            0x10
#define GY271_OSR_512           0x00
#define GY271_OSR_256           0x40
#define GY271_OSR_128           0x80
#define GY271_OSR_64            0xC0

#define GY271_FSR_2_RESOLUTION  16384.0f
#define GY271_FSR_8_RESOLUTION  4096.0f

// GY271(QMC5883L) Control Register 2
#define GY271_REG_CONTROL_2     0x0A

#define GY271_SOFT_RESET_BIT    0x80
#define GY271_ROL_PNT_EN_BIT    0x40
#define GY271_INT_ENB_BIT       0x01

// GY271(QMC5883L) SET/RESET Period Register
#define GY271_REG_PERIOD_FBR    0x0B

#define GY271_PERIOD_FBR_VAL    0x01

namespace DKVR
{
    namespace Driver
    {

        class ExtGY271QMC : public Magnetometer
        {
        public:
            ExtGY271QMC(const uint8_t parentAddress);

            uint8_t InitializeMag() override;
            uint8_t ReadRawMag(DataType::Vector3s &mag) override;

        private:
            const uint8_t parentAddress;
        };

    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_EXTGY271QMC
