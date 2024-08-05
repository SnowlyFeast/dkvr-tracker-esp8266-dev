#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t dkvr_udp_buffer[DKVR_NET_UDP_BUFFER_SIZE];

int dkvr_udp_is_server_up();

dkvr_err dkvr_udp_init();
dkvr_err dkvr_udp_server_begin(uint16_t local_server_port);
void dkvr_udp_server_stop();
int dkvr_udp_peek_recv();
void dkvr_udp_send_dgram(uint32_t remote_host_ip, uint16_t remote_host_port, const void* buffer, int len);

#ifdef __cplusplus
}
#endif