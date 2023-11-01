#include "MPU9250.h"

namespace DKVR
{
    namespace Driver
    {
        
//         MPU9250::MPU9250(const uint8_t deviceAddress) : MD4A(deviceAddress){};

//         uint8_t MPU9250::Initialize()
//         {
//             // setup argument
//             MDArguments arg;
//             arg.clksel = MPU_CLK_GYRO_Z_BIT;
//             arg.smprtDiv = 9;   
//             arg.dlpf = MPU_DLPF_42_HZ_BIT;
//             arg.gyroFsr = MPU_GYRO_FSR_2000_BIT;
//             arg.accelFsr = MPU_ACCEL_FSR_2_BIT;
//             arg.fifoEn = 0x0;
//             arg.intConfig = MPU_LATCH_INT_EN_BIT;
//             arg.intEnable = MPU_I2C_MST_INT_EN_BIT | MPU_DATA_RDY_EN_BIT;

//             // initialize mpu
//             uint8_t init = InitializeMPU(arg);
// #ifdef DKVR_ENABLE_IMU_DEBUG
//             PRINT(F("MPU init code : 0x"));
//             PRINTLN(String(init, HEX));
// #endif

//             I2C::ReadBytes(deviceAddress, 0x75, 1, &init);
//             PRINT("WAI : 0x");
//             PRINTLN(String(init, HEX));

//             init = InitializeMagnetometer();
// #ifdef DKVR_ENABLE_IMU_DEBUG
//             PRINT(F("Magnetometer init code : 0x"));
//             PRINTLN(String(init, HEX));
// #endif

//             return init;
//         };

//         uint8_t MPU9250::InitializeMagnetometer()
//         {
// #ifdef DKVR_IMU_MAG_AK8963
//             // enable bypass & disable I2C master
//             I2C::WriteBit(deviceAddress, MPU_REG_USER_CTRL, 5, 0x0);
//             I2C::WriteBit(deviceAddress, MPU_REG_INT_PIN_CFG, 1, 0x1);

//             // address scan
//             uint8_t data;
//             byte error, address; // variable for error and I2C address
//             int nDevices;

//             Serial.println("Scanning...");

//             nDevices = 0;
//             for (address = 1; address < 127; address++)
//             {
//                 // The i2c_scanner uses the return value of
//                 // the Write.endTransmisstion to see if
//                 // a device did acknowledge to the address.
//                 Wire.beginTransmission(address);
//                 error = Wire.endTransmission();

//                 if (error == 0)
//                 {
//                     Serial.print("I2C device found at address 0x");
//                     if (address < 16)
//                         Serial.print("0");
//                     Serial.print(address, HEX);
//                     Serial.println("  !");
//                     nDevices++;
//                 }
//                 else if (error == 4)
//                 {
//                     Serial.print("Unknown error at address 0x");
//                     if (address < 16)
//                         Serial.print("0");
//                     Serial.println(address, HEX);
//                 }
//             }
//             if (nDevices == 0)
//                 Serial.println("No I2C devices found\n");
//             else
//                 Serial.println("done\n");

//             // // AKM WIA
//             // uint8_t data;
//             // I2C::ReadBytes(AK8963_DEVICE_ADDRESS, AK8963_REG_WIA, 1, &data);
//             // Serial.print("Mag How I Am : 0x");
//             // Serial.println(String(data, HEX));


//             // disable bypass & enable I2C master
//             I2C::WriteBit(deviceAddress, MPU_REG_INT_PIN_CFG, 1, 0x0);
//             data = MPU_WAIT_FOR_ES_BIT | MPU_I2C_MST_CLK_400_BIT;
//             I2C::WriteBytes(deviceAddress, MPU_REG_I2C_MST_CTRL, 1, &data);
//             I2C::WriteBit(deviceAddress, MPU_REG_USER_CTRL, 5, 0x1);    // enable I2C_MST_EN

//             // setup I2C read mode
//             data = AK8963_DEVICE_ADDRESS | MPU_I2C_SLV0_R_MODE_BIT;
//             I2C::WriteBytes(deviceAddress, MPU_REG_I2C_SLV0_ADDR, 1, &data);
//             data = AK8963_REG_DATA_BEGIN;
//             I2C::WriteBytes(deviceAddress, MPU_REG_I2C_SLV0_REG, 1, &data);
//             data = MPU_I2C_SLV0_EN_BIT | AK8963_DATA_SIZE;
//             I2C::WriteBytes(deviceAddress, MPU_REG_I2C_SLV0_CTRL, 1, &data);

//             magnetometerReady = true;
//             return MPU_SUCCESS;
// #else
//             return MPU_ERR_NO_MAG_DEFINED;
// #endif
//         };

//         uint8_t MPU9250::GetRawMag(dkvr::datatype::Vector3s &mags)
//         {
//             if (!magInitialized)
//                 return MPU_ERR_MAG_INIT_FAIL;

//             if (I2C::ReadBytes(deviceAddress, MPU_REG_EXT_SENS_DATA, AK8963_DATA_SIZE, sharedBuffer))
//                 return MPU_ERR_READ_EXT_SENS_DATA;

//             mags.x = ((int16_t)sharedBuffer[1] << 8) | sharedBuffer[0];
//             mags.y = ((int16_t)sharedBuffer[3] << 8) | sharedBuffer[2];
//             mags.z = ((int16_t)sharedBuffer[5] << 8) | sharedBuffer[4];
//             return MPU_SUCCESS;
//         };

    } // namespace driver
    
} // namespace dkvr
