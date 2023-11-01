#ifndef DKVR_DRIVER_EXTGY271HMC
#define DKVR_DRIVER_EXTGY271HMC

#include "Driver/IMU.h"
#include "Driver/I2C.h"
#include "MPU6050.h"

// GY271(HMC5883L) Address and Data Reg
#define GY271_DEVICE_ADDRESS    0x1E
#define GY271_REG_DATA_BEGIN    0x03
#define GY271_DATA_SIZE         6

#define GY271_OP_READ           0x3D
#define GY271_OP_WRITE          0x3C


// GY271(HMC5883L) Configuration Register A
#define GY271_REG_CONF_A        0x00

#define GY271_MA_1              0x00
#define GY271_MA_2              0x20
#define GY271_MA_4              0x40
#define GY271_MA_8              0x60
#define GY271_DO_3HZ            0x08
#define GY271_DO_7P5HZ          0x0C
#define GY271_DO_15HZ           0x10
#define GY271_DO_30HZ           0x14
#define GY271_DO_75HZ           0x18
#define GY271_MS_NORMAL         0x00
#define GY271_MS_POS_BIAS       0x01
#define GY271_MS_NEG_BIAS       0x02


// GY271(HMC5883L) Configuration Register B
#define GY271_REG_CONF_B        0x01

#define GY271_GN_2P5G           0x60
#define GY271_GN_4G             0x80
#define GY271_GN_5P6G           0xC0
#define GY271_GN_8P1G           0xE0

#define GY271_GN_2P5_RESOLUTION     660.0f
#define GY271_GN_4G_RESOLUTION      440.0f
#define GY271_GN_5P6G_RESOLUTION    330.0f
#define GY271_GN_8P1G_RESOLUTION    230.0f


// GY271(HMC5883L) Mode Register
#define GY271_REG_MODE          0x02

#define GY271_HS_ENABLE         0x80
#define GY271_MD_CONTINUOUS     0x00
#define GY271_MD_SINGLE         0x01
#define GY271_MD_IDLE           0x10


// GY271(HMC5883L) Status Register
#define GY271_REG_STATUS        0x09



namespace DKVR
{
    namespace Driver
    {

        class ExtGY271HMC : public Magnetometer
        {
        public:
            ExtGY271HMC(const uint8_t parentAddress);

            uint8_t InitializeMag() override;
            uint8_t ReadRawMag(DataType::Vector3s &mag) override;

        private:
            const uint8_t parentAddress;
        };

    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_EXTGY271HMC
