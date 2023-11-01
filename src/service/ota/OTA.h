#ifndef DKVR_SERVICE_OTA
#define DKVR_SERVICE_OTA

#include <ArduinoOTA.h>

#define DKVR_OTA_TIMEOUT    60000UL

namespace DKVR
{
    namespace Service
    {
        class OTAService
        {
        public:
            void Initialize(uint16_t port = 0, const char *hostname = nullptr, const char *password = nullptr);
            void Update();
            
        private:
            bool enabled = true;
        };

    } // namespace service

} // namespace dkvr

#endif // DKVR_SERVICE_OTA
