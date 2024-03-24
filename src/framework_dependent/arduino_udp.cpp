#include "common/udp_interface.h"

#include <WiFiUdp.h>

#include "common/dkvr_core.h"

udp_buffer_t udp_buffer{};

static udp_server_status_t udp_server_status = UDP_SERVER_STATUS_CLOSED;
static WiFiUDP udp;
static IPAddress host_ip;
static uint16_t host_port = 0;
static uint16_t local_port = 0;

udp_server_status_t get_udp_server_status()
{
    return udp_server_status;
}

dkvr_err_t dkvr_udp_init()
{
    return DKVR_OK;
}

void dkvr_udp_setup(uint32_t remote_host_ip, uint16_t remote_host_port, uint16_t local_server_port)
{
    host_ip = remote_host_ip;
    host_port = remote_host_port;
    local_port = local_server_port;
}

dkvr_err_t begin_udp_server()
{
    uint8_t result = udp.begin(local_port);
    if (result)
    {
        udp_server_status = UDP_SERVER_STATUS_LISTENING;
        PRINTLN("UDP server opened.");
    }
    return result ? DKVR_OK : DKVR_ERR_UDP_BIND_FAIL;
}

void stop_udp_server()
{
    if (udp_server_status == UDP_SERVER_STATUS_LISTENING) {
        udp.stop();
        udp_server_status = UDP_SERVER_STATUS_CLOSED;
        PRINTLN("UDP server closed.");
    }
}

int peek_udp_recv()
{
    if (udp_server_status == UDP_SERVER_STATUS_CLOSED)
        return 0;

    int len = udp.parsePacket();
    if (len == 0)
        return 0;
    
    memset(udp_buffer.buffer, 0, DKVR_NET_UDP_BUFFER_SIZE);
    len = udp.read(udp_buffer.buffer, DKVR_NET_UDP_BUFFER_SIZE);

#ifdef DKVR_DEBUG_NET_RECV
    PRINT("Packet recv :");
    for (int i = 0; i < len; i++){
        if (!(i % 4))
            PRINT(" ");
        PRINT((udp_buffer.buffer[i]));
    }
    PRINTLN(" (end)");
#endif

    return len;
}

void send_udp(const void* buffer, int len)
{
    udp.beginPacket(host_ip, host_port);
    udp.write(reinterpret_cast<const uint8_t*>(buffer), len);
    udp.endPacket();
#ifdef DKVR_DEBUG_NET_SEND
    PRINT("Packet sent :");
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(buffer);
    for (int i = 0; i < len; i++){
        if (!(i % 4))
            PRINT(" ");
        PRINT((ptr[i]));
    }
    PRINTLN(" (end)");
#endif
}