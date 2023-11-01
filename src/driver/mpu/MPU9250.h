#ifndef DKVR_DRIVER_MPU9250
#define DKVR_DRIVER_MPU9250



#define MPU9250_DEVICE_ADDRESS_A0_LOW   0x68
#define MPU9250_DEVICE_ADDRESS_A0_HIGH  0x69

// AK8963 Address and Data Reg
#define AK8963_DEVICE_ADDRESS   0x0C
#define AK8963_REG_DATA_BEGIN   0x03
#define AK8963_DATA_SIZE        6

// AK8963 WIA
#define AK8963_REG_WIA          0x00


namespace DKVR
{
    namespace Driver
    {

        // class MPU9250 : public MD4A
        // {
        // public:
        //     MPU9250(const uint8_t deviceAddress);
        //     uint8_t Initialize() override;
        //     uint8_t InitializeMagnetometer() override;
        //     uint8_t GetRawMag(DataType::Vector3s &mags) override;
        // };

    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_MPU9250
