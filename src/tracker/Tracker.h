#ifndef DKVR_TRACKER_TRACKER
#define DKVR_TRACKER_TRACKER

#include "configuration.h"
#include "TrackerConfiguration.h"
#include "TrackerStatus.h"
#include "Utils.h"
#include "driver/IMU.h"
#include "driver/factory/IMUFactory.h"
#include "service/network/Network.h"
#include "service/led/LED.h"
#include "service/ahrs/OrientationFixer.h"


#define OPCODE_LOCATE       0x11
#define OPCODE_PING         0x12
#define OPCODE_PONG         0x13

#define OPCODE_CONFIGURE    0x22
#define OPCODE_ACTIVATE     0x23
#define OPCODE_DEACTIVATE   0x24

#define OPCODE_IMU_DATA     0x32

using namespace DKVR::Service;

namespace DKVR
{
    namespace Core
    {

        class Tracker
        {
        public:
            Tracker() : network(DKVR_WIFI_SSID, DKVR_WIFI_PASSWROD, DKVR_HOST_IP, DKVR_HOST_PORT) {}
            unsigned int Initialize();
            void Update();
            void OnInterrupt();

        private:
            void UpdateNetwork();
            void HandlePacket();
            void UpdateLEDMode();
            void SendIMUData();

            bool IsConnected() { return network.GetStatus() == HostConnected; }

            TrackerConfiguration config;
            NetworkService network;
            LEDService led;

            Driver::IMU *imu = Driver::IMUFactory::CreateAutoConfiguredIMU();
            Utils::SimpleTimer idleTimer;
            Utils::SimpleTimer imuTimer;

        };

    } // namespace core

} // namespace dkvr


#endif // DKVR_TRACKER_TRACKER
