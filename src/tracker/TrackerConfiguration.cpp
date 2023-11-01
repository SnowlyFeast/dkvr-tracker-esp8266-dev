#include "TrackerConfiguration.h"

namespace DKVR
{
    namespace Core
    {

        void TrackerConfiguration::Reset()
        {
            behavior = Behavior();
            calibMatAcc = CalibrationMatrix();
        }

        void TrackerConfiguration::Configure(unsigned char key, const unsigned char *value)
        {
            switch (key)
            {
            case CONF_KEY_BEHAVIOR:
                behavior.active = (*value) & CONF_BEHAVIOR_ACTIVE;
                behavior.raw = (*value) & CONF_BEHAVIOR_RAW;
                behavior.led = (*value) & CONF_BEHAVIOR_DISABLE_LED;
                break;

            case CONF_KEY_ACC_CALIB_X:
                memcpy(&calibMatAcc.columnX, value, sizeof(float) * 4);
                break;

            case CONF_KEY_ACC_CALIB_Y:
                memcpy(&calibMatAcc.columnY, value, sizeof(float) * 4);
                break;

            case CONF_KEY_ACC_CALIB_Z:
                memcpy(&calibMatAcc.columnZ, value, sizeof(float) * 4);
                break;
            }

#ifdef DKVR_ENABLE_CONFIG_DEBUG
            switch (key)
            {
            case CONF_KEY_BEHAVIOR:
                PRINT("Behavior is set to : 0b");
                PRINTLN(String(*value, BIN));
                return;

            case CONF_KEY_ACC_CALIB_X:
                PRINT("Accel Calibration Matrix Column Vector 1 is set to : ");
                PRINT(String(calibMatAcc.columnX.x, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnX.y, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnX.z, 4));
                PRINT(" ");
                PRINTLN(String(calibMatAcc.columnX.w, 4));
                break;

            case CONF_KEY_ACC_CALIB_Y:
                PRINT("Accel Calibration Matrix Column Vector 2 is set to : ");
                PRINT(String(calibMatAcc.columnY.x, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnY.y, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnY.z, 4));
                PRINT(" ");
                PRINTLN(String(calibMatAcc.columnY.w, 4));
                break;

            case CONF_KEY_ACC_CALIB_Z:
                PRINT("Accel Calibration Matrix Column Vector 3 is set to : ");
                PRINT(String(calibMatAcc.columnZ.x, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnZ.y, 4));
                PRINT(" ");
                PRINT(String(calibMatAcc.columnZ.z, 4));
                PRINT(" ");
                PRINTLN(String(calibMatAcc.columnZ.w, 4));
                break;
            }
#endif
        }

        void TrackerConfiguration::CalibrateAccel(DataType::Vector3f &accel)
        {
            float x = calibMatAcc.columnX.Dot(accel);
            float y = calibMatAcc.columnY.Dot(accel);
            float z = calibMatAcc.columnZ.Dot(accel);
            accel.x = x;
            accel.y = y;
            accel.z = z;
        }

    } // namespace Core

} // namespace DKVR
