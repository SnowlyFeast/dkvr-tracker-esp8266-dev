#ifndef DKVR_DRIVER_IMU
#define DKVR_DRIVER_IMU

#include "configuration.h"
#include "DataType.h"


// Sensor Code
#define IMU_SENSOR_GYRO     0x00
#define IMU_SENSOR_ACCEL    0x40
#define IMU_SENSOR_MAG      0x80

// Success
#define IMU_SUCCESS                 0x00

// Initialization Error
#define IMU_INIT_STANDBY            0x01
#define IMU_INIT_INVALID_ARGS       0x02
#define IMU_INIT_NACK_RECEIVED      0x03
#define IMU_INIT_SELF_TEST_FAIL     0x04
#define IMU_INIT_EXT_NACK_RECEIVED  0x05

// I2C Error
#define IMU_I2C_ERROR               0x10
#define IMU_INT_READ_FAIL           0x11
#define IMU_GYRO_READ_FAIL          0x12
#define IMU_ACCEL_READ_FAIL         0x13
#define IMU_MAG_READ_FAIL           0x14

namespace DKVR
{
    namespace Driver
    {

        class Gyroscope
        {
        public:
            virtual uint8_t InitializeGyro() = 0;
            virtual uint8_t ReadRawGyro(DataType::Vector3s &gyro) = 0;

            float GetGyroSentitivity() { return gyroSensitivity; }

        protected:
            float gyroSensitivity = 32768.0f;
        };

        class Accelerometer
        {
        public:
            virtual uint8_t InitializeAccel() = 0;
            virtual uint8_t ReadRawAccel(DataType::Vector3s &accel) = 0;

            float GetAccelSensitivity() { return accelSensitivity; }

        protected:
            float accelSensitivity = 32768.0f;
        };

        class Magnetometer
        {
        public:
            virtual uint8_t InitializeMag() = 0;
            virtual uint8_t ReadRawMag(DataType::Vector3s &mag) = 0;
            
            float GetMagSensitivity() { return magSensitivity; }

        protected:
            float magSensitivity = 32768.0f;
        };

        class Interruptible
        {
        public:
            virtual uint8_t ReadInterrupt(uint8_t &intout) = 0;
            virtual void HandleInterrupt(uint8_t interrupt) = 0;
            virtual bool GetDataReadyInterrupt() = 0;
        };

        class IMU
        {
        public:
            IMU(Gyroscope *gyro, Accelerometer *accel, Magnetometer *mag, Interruptible *interrupt = nullptr);

            uint8_t Initialize();
            uint8_t ReadAllSensors();
            uint8_t ReadAndHandleInterrupt();
            virtual bool IsDataReady() { return (interrupt ? interrupt->GetDataReadyInterrupt() : false); };

            uint8_t GetInitResult() const { return initResult; }
            bool IsInterruptible() const { return interrupt; }

            DataType::Vector3f gyro;
            DataType::Vector3f accel;
            DataType::Vector3f mag;

        private:
            Gyroscope *sensorGyro;
            Accelerometer *sensorAccel;
            Magnetometer *sensorMag;
            Interruptible *interrupt;

            uint8_t initResult;
        };

    } // namespace driver

} // namespace dkvr

#endif // DKVR_DRIVER_IMU
