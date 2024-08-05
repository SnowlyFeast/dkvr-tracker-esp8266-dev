#include "common/udp_interface.h"

#include <WiFiUdp.h>

#include "common/dkvr_core.h"

uint8_t dkvr_udp_buffer[DKVR_NET_UDP_BUFFER_SIZE];

static WiFiUDP udp;
static int server_up = 0;

int dkvr_udp_is_server_up()
{
    return server_up;
}

dkvr_err dkvr_udp_init()
{
    return DKVR_OK;
}

dkvr_err dkvr_udp_server_begin(uint16_t local_server_port)
{
    if (server_up)
        return DKVR_OK;

    uint8_t result = udp.begin(local_server_port);
    if (result)
    {
        server_up = 1;
        PRINTLN("UDP server opened.");
    }

    return result ? DKVR_OK : DKVR_ERR_UDP_BIND_FAIL;
}

void dkvr_udp_server_stop()
{
    if (server_up) {
        udp.stop();
        server_up = 0;
        PRINTLN("UDP server closed.");
    }
}

int dkvr_udp_peek_recv()
{
    if (!server_up)
        return 0;

    int len = udp.parsePacket();
    if (len == 0)
        return 0;
    
    memset(dkvr_udp_buffer, 0, DKVR_NET_UDP_BUFFER_SIZE);
    len = udp.read(dkvr_udp_buffer, DKVR_NET_UDP_BUFFER_SIZE);

#ifdef DKVR_DEBUG_NET_RECV
    PRINT("Packet recv :");
    for (int i = 0; i < len; i++){
        if (!(i % 4))
            PRINT(" ");
        PRINT((dkvr_udp_buffer[i]));
    }
    PRINTLN(" (end)");
#endif

    return len;
}

void dkvr_udp_send_dgram(uint32_t remote_host_ip, uint16_t remote_host_port, const void* buffer, int len)
{
    udp.beginPacket(remote_host_ip, remote_host_port);
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
