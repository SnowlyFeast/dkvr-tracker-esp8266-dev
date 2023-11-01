#include "IMU.h"

namespace DKVR
{
    namespace Driver
    {

        IMU::IMU(Gyroscope *gyro, Accelerometer *accel, Magnetometer *mag, Interruptible *interrupt)
            : sensorGyro(gyro), sensorAccel(accel), sensorMag(mag), interrupt(interrupt), initResult(IMU_INIT_STANDBY)
        {
            // check missing arguments
            if (!gyro)
                initResult = IMU_SENSOR_GYRO | IMU_INIT_INVALID_ARGS;
            else if (!accel)
                initResult = IMU_SENSOR_ACCEL | IMU_INIT_INVALID_ARGS;
            else if (!mag)
                initResult = IMU_SENSOR_MAG | IMU_INIT_INVALID_ARGS;
        }

        uint8_t IMU::Initialize()
        {
            // check double init
            if (initResult != IMU_INIT_STANDBY)
                return initResult;

            // initialize each sonsor
            if ((initResult = sensorGyro->InitializeGyro()))
                return (initResult |= IMU_SENSOR_GYRO);

            if ((initResult = sensorAccel->InitializeAccel()))
                return (initResult |= IMU_SENSOR_ACCEL);
                
            if ((initResult = sensorMag->InitializeMag()))
                return (initResult |= IMU_SENSOR_MAG);

            return initResult;
        }

        uint8_t IMU::ReadAllSensors()
        {
            uint8_t result;
            DataType::Vector3s sGyro, sAccel, sMag;

            if ((result = sensorGyro->ReadRawGyro(sGyro)))
                return IMU_SENSOR_GYRO | result;
            if ((result = sensorAccel->ReadRawAccel(sAccel)))
                return IMU_SENSOR_ACCEL | result;
            if ((result = sensorMag->ReadRawMag(sMag)))
                return IMU_SENSOR_MAG | result;

            gyro = sGyro.ToVector3f(sensorGyro->GetGyroSentitivity());
            accel = sAccel.ToVector3f(sensorAccel->GetAccelSensitivity());
            mag = sMag.ToVector3f(sensorMag->GetMagSensitivity());

            return IMU_SUCCESS;
        }

        uint8_t IMU::ReadAndHandleInterrupt()
        {
            unsigned char code;
            if (!interrupt || interrupt->ReadInterrupt(code))
                return IMU_INT_READ_FAIL;
            interrupt->HandleInterrupt(code);
            return IMU_SUCCESS;
        }

    } // namespace Driver
    
} // namespace DKVR
