#include "OTA.h"

namespace DKVR
{
    namespace Service
    {
        void OnStart()
        {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_FS
                type = "filesystem";

            // NOTE: if updating FS this would be the place to unmount FS using FS.end()
            Serial.println(F("[OTA]Start updating ") + type);
        };

        void OnEnd()
        {
            Serial.println(F("\nEnd."));
        };

        void OnProgress(unsigned int progress, unsigned int total)
        {
            Serial.printf("[OTA]Progress: %u%%\r", (progress / (total / 100)));
        };

        void OnError(ota_error_t error)
        {
            Serial.printf("[OTA]Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println(F("Auth Failed"));
            else if (error == OTA_BEGIN_ERROR)
                Serial.println(F("Begin Failed"));
            else if (error == OTA_CONNECT_ERROR)
                Serial.println(F("Connect Failed"));
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println(F("Receive Failed"));
            else if (error == OTA_END_ERROR)
                Serial.println(F("End Failed"));
        };
        
        void OTAService::Initialize(uint16_t port, const char *hostname, const char *password)
        {
            ArduinoOTA.setPort(port);
            ArduinoOTA.setHostname(hostname);
            ArduinoOTA.setPassword(password);

            ArduinoOTA.onStart(OnStart);
            ArduinoOTA.onEnd(OnEnd);
            ArduinoOTA.onProgress(OnProgress);
            ArduinoOTA.onError(OnError);

            ArduinoOTA.begin();
        };

        void OTAService::Update()
        {
            if (!enabled)
                return;

            // disable OTA after 60 secs
            if (millis() > DKVR_OTA_TIMEOUT)
                enabled = false;

            ArduinoOTA.handle();
        };

    } // namespace service

} // namespace dkvr
