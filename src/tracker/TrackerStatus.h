#ifndef DKVR_TRACKER_TRACKERSTATUS
#define DKVR_TRACKER_TRACKERSTATUS



#define TRACKER_CONF_ACTIVE     0x01
#define TRACKER_CONF_RAW        0x02
#define TRACKER_CONF_LED_OFF    0x04


namespace DKVR
{
    namespace Service
    {
        
        class TrackerStatus {
        public:
            unsigned char EncodeConfiguration() const;
            void DecodeConfiguraiton(unsigned char config);
            void ResetConfiguration();

            void SetActive(bool active) { this->active = active; }
            void SetRawMode(bool rawMode) { this->rawMode = rawMode; }
            void SetLEDOff(bool ledOff) { this->ledOff = ledOff; }

            bool IsActive() const { return active; }
            bool IsRawMode() const { return rawMode; }
            bool IsLEDOff() const { return ledOff; }

        private:
            bool active;
            bool rawMode;
            bool ledOff;
            unsigned char error;
            unsigned short battery;
        };

    } // namespace Tracker
    
} // namespace DKVR


#endif // DKVR_TRACKER_TRACKERSTATUS
