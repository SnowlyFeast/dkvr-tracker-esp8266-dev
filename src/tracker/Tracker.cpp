#include "Tracker.h"

namespace DKVR
{
    namespace Core
    {
        
        unsigned int Tracker::Initialize() {
            led.SetMode(Manual);
            led.SetLED(true);
            
            // initialize IMU
            unsigned char imuResult = imu->Initialize();
            if (imuResult) {
                PRINT("IMU initialization failed : 0x");
                PRINTLN(String(imuResult, HEX));
            }

            // initialize network
            unsigned int wifi = network.ConnectToWiFi();
            unsigned int host = network.ConnectToHost();
            if (wifi)
            {
                PRINT("WiFi connection failed : 0x");
                PRINTLN(String(wifi, HEX));
            }
            else if (host)
            {
                PRINT("Host connection failed : 0x");
                PRINTLN(String(host, HEX));
            }
            
            // report error to host if possible
            if (imuResult && !host) {
                // TODO: send Tracker Status to host
            }

            led.SetMode((imuResult || host) ? Fastest : Slowest);
            return imuResult;
        }

        void Tracker::Update()
        {
            if (idleTimer.IsOver())
            {
                UpdateNetwork();
                HandlePacket();

                // Non-interruptible IMU
                if (!imu->IsInterruptible() && imuTimer.IsOver()) {
                    imuTimer.Set(1000 / DKVR_IMU_SAMPLING_RATE);
                    SendIMUData();
                }
            }

            led.Update();
        }

        void Tracker::UpdateNetwork()
        {
            switch (network.UpdateConnectionStatus())
            {
            case Invalid:
                // nothing to do
                led.SetMode(Fastest);
                idleTimer.Set(10000);
                break;

            case Disconnected:
                led.SetMode(Manual);
                led.SetLED(true);
                if (network.ConnectToWiFi())
                {
                    PRINTLN("WiFi connection failed. Retrying after 5secs.");
                    led.SetMode(Fastest);
                    idleTimer.Set(5000);
                    break;
                }
                // continue

            case WiFiConnected:
                led.SetMode(Manual);
                led.SetLED(true);
                if (network.ConnectToHost())
                {
                    PRINTLN("Host connection failed. Retrying after 5secs.");
                    led.SetMode(Fastest);
                    idleTimer.Set(5000);
                    break;
                }

                // host connected
                config.Reset();
                led.SetMode(Slowest);
                break;

            case HostConnected:
                break;
            }
        }

        void Tracker::HandlePacket()
        {
            while (network.ReadPacket())
            {
                const unsigned char *buffer = network.GetBuffer();
                switch (PACKET_OPCODE(buffer))
                {
                case OPCODE_LOCATE:
                    led.Locate();
                    break;

                case OPCODE_PING:
                    network.SendPacket(OPCODE_PONG, nullptr, 0);
                    break;


                case OPCODE_CONFIGURE:
                {
                    unsigned char key = PACKET_PAYLOAD_B(buffer);
                    config.Configure(key, PACKET_PAYLOAD(buffer) + 1);
                    network.SendPacket(OPCODE_CONFIGURE, &key, 1);
                    UpdateLEDMode();
                }
                    break;

                case OPCODE_ACTIVATE:
                    config.SetActive(true);
                    network.SendPacket(OPCODE_ACTIVATE, nullptr, 0);
                    UpdateLEDMode();
                    break;
                
                case OPCODE_DEACTIVATE:
                    config.SetActive(false);
                    network.SendPacket(OPCODE_DEACTIVATE, nullptr, 0);
                    UpdateLEDMode();
                    break;

                default:
                    break;
                }
            }
        }

        void Tracker::UpdateLEDMode()
        {
            if (config.IsLEDDisabled()) {
                led.SetMode(Manual);
                led.SetLED(false);
            }
            else {
                led.SetMode( config.IsActive() ? Normal : Slowest);
            }
        }

        void Tracker::OnInterrupt()
        {
            if (imu->ReadAndHandleInterrupt()) {
                PRINTLN("Failed to read interrupt.");
                return;
            }

            if (imu->IsDataReady())
                SendIMUData();
        }

        void Tracker::SendIMUData()
        {
            if (!IsConnected() || !config.IsActive())
                return;

            // fix orientation and calibrate
            imu->ReadAllSensors();
            OrientationFixer::FixGyro(imu->gyro);
            OrientationFixer::FixAccel(imu->accel);
            OrientationFixer::FixMag(imu->mag);
            config.CalibrateAccel(imu->accel);

            if (config.IsRawMode())
            {
                unsigned char buf[36];
                memcpy(buf, &imu->gyro, sizeof(DataType::Vector3f));
                memcpy(buf + 12, &imu->accel, sizeof(DataType::Vector3f));
                memcpy(buf + 24, &imu->mag, sizeof(DataType::Vector3f));
                network.SendPacket(OPCODE_IMU_DATA, buf, 36);
            }
            else
            {
                // TOOD: Begin Kalman Filtering
                unsigned char buf[16];
                float quat = 1.0f;
                for (int i = 0; i < 4; i++)
                    memcpy(buf + i * 4, &quat, sizeof(float));
                network.SendPacket(OPCODE_IMU_DATA, buf, 16);
            }

#ifdef DKVR_ENABLE_DIGEST_ANALYSIS
            static int count = 0;
            static unsigned long time = millis();
            if (count >= DKVR_IMU_SAMPLING_RATE)
            {
                count = 0;
                PRINT("IMU Data Digestion Rate : ");
                PRINTLN(100000 / (millis() - time));
                time = millis();
            }
            count++;
#endif
        }

    } // namespace core

} // namespace dkvr
