#include "TrackerStatus.h"

namespace DKVR
{
    namespace Service
    {
        
        unsigned char TrackerStatus::EncodeConfiguration() const {
            unsigned char conf = 0x0;
            
            if (active)
                conf |= TRACKER_CONF_ACTIVE;
            
            if (rawMode)
                conf |= TRACKER_CONF_RAW;

            if (ledOff)
                conf |= TRACKER_CONF_LED_OFF;

            return conf;
        }

        void TrackerStatus::DecodeConfiguraiton(unsigned char config)
        {
            active = config & TRACKER_CONF_ACTIVE;
            rawMode = config & TRACKER_CONF_RAW;
            ledOff = config & TRACKER_CONF_LED_OFF;
        }

        void TrackerStatus::ResetConfiguration()
        {
            active = false;
            rawMode = false;
            ledOff = false;
        }

    } // namespace Service
    
} // namespace DKVR
