#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum udp_server_status_e
{
    UDP_SERVER_STATUS_CLOSED,
    UDP_SERVER_STATUS_LISTENING
} udp_server_status_t;

typedef struct udp_buffer_s
{
    uint8_t buffer[DKVR_NET_UDP_BUFFER_SIZE];
} udp_buffer_t;


extern udp_buffer_t udp_buffer;

udp_server_status_t get_udp_server_status();

dkvr_err_t dkvr_udp_init();
void dkvr_udp_setup(uint32_t remote_host_ip, uint16_t remote_host_port, uint16_t local_server_port);

dkvr_err_t begin_udp_server();
void stop_udp_server();
int peek_udp_recv();
void send_udp(const void* buffer, int len);

#ifdef __cplusplus
}
#endif