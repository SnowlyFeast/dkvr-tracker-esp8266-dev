#include "Network.h"

namespace DKVR
{
    namespace Service
    {

        NetworkService::NetworkService(const char *ssid, const char *password, const char *host, unsigned short port)
            : ssid(ssid), password(password), port(port)
        {
            if (this->host.fromString(host))
                status = Disconnected;
        }

        unsigned int NetworkService::ConnectToWiFi()
        {
            if (status == Invalid)
                return NETWORK_ERR_BAD_HOST_NAME;
            else if (status != Disconnected)
                return 0;   // already connected

            WiFi.mode(WIFI_STA);
            WiFi.begin(ssid, password);

            if (WiFi.waitForConnectResult() != WL_CONNECTED)
                return NETWORK_ERR_NO_WIFI;
            
            PRINT("WiFi connected with IP : ");
            PRINTLN(WiFi.localIP());
            status = WiFiConnected;

            return UDP.begin(NETWORK_UDP_LISTEN_PORT) ? 0 : NETWORK_ERR_NO_SOCKET_AVAILABLE;
        }

        unsigned int NetworkService::ConnectToHost()
        {
            if (status != WiFiConnected)
                return NETWORK_ERR_NO_WIFI;

            // try handshake
            bool success = false;
            for (int retry = 5; retry > 0; retry--)
            {
                SendPacket(OPCODE_HANDSHAKE1, nullptr, 0);
                unsigned long timestamp = millis();
                while (!success && (timestamp + NETWORK_RESPONSE_TIMEOUT > millis()))
                {
                    success = ReadPacket();
                }

                if (success)
                    break;
            }

            if (!success)
                return NETWORK_ERR_NO_HOST;

            if (PACKET_OPCODE(packetBuffer) != OPCODE_HANDSHAKE2)
                return NETWORK_ERR_WRONG_RESPONSE;

            status = HostConnected;
            lastHbRecv = millis();
            SendHeartbeat();
            return 0;
        }

        NetworkStatus NetworkService::UpdateConnectionStatus()
        {
            switch (status)
            {
            default:
            case Disconnected:
                break;

            case WiFiConnected:
                // keep check wifi status
                if (WiFi.status() != WL_CONNECTED)
                {
                    status = Disconnected;
                    UDP.stop();
                    WiFi.disconnect();
                }
                break;

            case HostConnected:
                {
                    unsigned long now = millis();
                    // host-side heartbeat
                    if (now > lastHbRecv + NETWORK_HEARTBEAT_TIMEOUT)
                    {
                        status = WiFiConnected;
                        break;
                    }

                    // client-side heartbeat
                    if (now > lastHbSent + NETWORK_HEARTBEAT_INTERVAL)
                        SendHeartbeat();
                }
                break;
            }
            return status;
        }

        void NetworkService::SendPacket(unsigned char opcode, const void *payload, size_t size)
        {
            dataLength = size + 8;

            PACKET_HEADER1(packetBuffer) = 'D';
            PACKET_HEADER2(packetBuffer) = 'K';
            PACKET_OPCODE(packetBuffer)  = opcode;
            memcpy(PACKET_NUMBER(packetBuffer), &(++packetNumber), sizeof(unsigned long));

            if (payload)
                memcpy(PACKET_PAYLOAD(packetBuffer), payload, size);

            unsigned char parities = 0;
            for (size_t i = 0; i < size + 7; i++)
                parities ^= packetBuffer[i];
            PACKET_PARITY(packetBuffer, dataLength) = parities;

            UDP.beginPacket(host, port);
            UDP.write(packetBuffer, dataLength);
            UDP.endPacket();

#ifdef DKVR_ENABLE_NETWORK_SEND_DEBUG
            PRINT("Packet Sent :");
            for (int i = 0; i < dataLength; i++)
            {
                PRINT(" ");
                PRINT(String(packetBuffer[i], HEX));
            }
            PRINTLN(" (end)");
#endif
        }

        bool NetworkService::ReadPacket()
        {
            unsigned char temp = UDP.parsePacket();
            if (!temp)
                return false;

            dataLength = UDP.read(packetBuffer, NETWORK_SHARED_BUFFUER_SIZE);
#ifdef DKVR_ENABLE_NETWORK_RECV_DEBUG
            PRINT("Packet Recv :");
            for (int i = 0; i < dataLength; i++)
            {
                PRINT(" ");
                PRINT(String(packetBuffer[i], HEX));
            }
            PRINTLN(" (end)");
#endif
            if (PACKET_HEADER1(packetBuffer) != 'D' || PACKET_HEADER2(packetBuffer) != 'K')
                return false;

            temp = 0;
            for (int i = 0; i < dataLength - 1; i++)
                temp ^= packetBuffer[i];
            if (temp != PACKET_PARITY(packetBuffer, dataLength))
                return false;

            // intercept heartbeat
            if (PACKET_OPCODE(packetBuffer) == OPCODE_HEARTBEAT) {
                lastHbRecv = millis();
                return false;
            }

            return true;
        }

        void NetworkService::SendHeartbeat()
        {
            if (status != HostConnected)
                return;

            SendPacket(OPCODE_HEARTBEAT, nullptr, 0);
            lastHbSent = millis();
        }


    } // namespace service

} // namespace dkvr
