#ifndef DKVR_TRACKER_CONFIGURATION
#define DKVR_TRACKER_CONFIGURATION

#include <Arduino.h>
#include "driver/DataType.h"
#include "configuration.h"

#define CONF_KEY_BEHAVIOR       0x00
#define CONF_KEY_ACC_CALIB_X    0x01
#define CONF_KEY_ACC_CALIB_Y    0x02
#define CONF_KEY_ACC_CALIB_Z    0x03

#define CONF_BEHAVIOR_ACTIVE        0x01
#define CONF_BEHAVIOR_RAW           0x02
#define CONF_BEHAVIOR_DISABLE_LED   0x04

namespace DKVR
{
    namespace Core
    {

        struct Behavior
        {
            bool active = false;
            bool raw = false;
            bool led = false;

            unsigned char Encode()
            {
                return (active * CONF_BEHAVIOR_ACTIVE) | (raw * CONF_BEHAVIOR_RAW) | (led * CONF_BEHAVIOR_DISABLE_LED);
            }
        };

        struct CalibrationMatrix
        {
            DataType::Vector4f columnX = DataType::Vector4f(1.0f, 0.0f, 0.0f, 0.0f);
            DataType::Vector4f columnY = DataType::Vector4f(0.0f, 1.0f, 0.0f, 0.0f);
            DataType::Vector4f columnZ = DataType::Vector4f(0.0f, 0.0f, 1.0f, 0.0f);
        };

        class TrackerConfiguration
        {
        public:
            void Reset();
            void Configure(unsigned char key, const unsigned char *value);
            void CalibrateAccel(DataType::Vector3f &accel);

            void SetActive(bool active) { behavior.active = active; }
            bool IsActive() { return behavior.active; }
            bool IsRawMode() { return behavior.raw; }
            bool IsLEDDisabled() { return behavior.led; }

        private:
            Behavior behavior;
            CalibrationMatrix calibMatAcc;
        };

    } // namespace Core

} // namespace dkvr


#endif // DKVR_TRACKER_CONFIGURATION
