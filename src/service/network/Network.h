#ifndef DKVR_SERVICE_NETWORK
#define DKVR_SERVICE_NETWORK

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "configuration.h"

// Network Service Error Code
#define NETWORK_SUCCESS                 0x00
#define NETWORK_ERR                     0x40
#define NETWORK_ERR_BAD_HOST_NAME       0x41
#define NETWORK_ERR_NO_WIFI             0x42
#define NETWORK_ERR_NO_SOCKET_AVAILABLE 0x43
#define NETWORK_ERR_NO_HOST             0x44
#define NETWORK_ERR_WRONG_RESPONSE      0x45

// Configurations
#define NETWORK_SHARED_BUFFUER_SIZE 64
#define NETWORK_UDP_LISTEN_PORT     8899
#define NETWORK_RESPONSE_TIMEOUT    500
#define NETWORK_HEARTBEAT_INTERVAL  1000
#define NETWORK_HEARTBEAT_TIMEOUT   5000

// Packet Buffer Macros
#define PACKET_HEADER1(bf)      bf[0]
#define PACKET_HEADER2(bf)      bf[1]
#define PACKET_NUMBER(bf)       &bf[2]
#define PACKET_OPCODE(bf)       bf[6]
#define PACKET_PAYLOAD(bf)      &bf[7]
#define PACKET_PAYLOAD_B(bf)    bf[7]
#define PACKET_PARITY(bf, size) bf[size - 1]

// Networking OpCode
#define OPCODE_HANDSHAKE1   0x01
#define OPCODE_HANDSHAKE2   0x02
#define OPCODE_HEARTBEAT    0x03


namespace DKVR
{
    namespace Service
    {
        enum NetworkStatus
        {
            Invalid,
            Disconnected,
            WiFiConnected,
            HostConnected
        };

        class NetworkService
        {
        public:
            NetworkService(const char *ssid, const char *password, const char *host, unsigned short port);

            unsigned int ConnectToWiFi();
            unsigned int ConnectToHost();
            NetworkStatus UpdateConnectionStatus();

            void SendPacket(unsigned char opcode, const void *payload, size_t size);
            bool ReadPacket();

            NetworkStatus GetStatus() const { return status; }
            const unsigned char *GetBuffer() const { return packetBuffer; }

        private:
            void SendHeartbeat();

            const char *ssid;
            const char *password;
            IPAddress host;
            unsigned short port;
            WiFiUDP UDP;

            NetworkStatus status = Invalid;
            unsigned char packetBuffer[NETWORK_SHARED_BUFFUER_SIZE];
            unsigned char dataLength = 0;
            unsigned long packetNumber = 0;
            unsigned long lastHbSent = 0;
            unsigned long lastHbRecv = 0;
        };

    } // namespace service

} // namespace dkvr

#endif // DKVR_SERVICE_NETWORK
