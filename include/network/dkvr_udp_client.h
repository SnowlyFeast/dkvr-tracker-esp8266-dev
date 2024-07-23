#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum client_status_e
{
    CLIENT_DISCONNECTED,
    CLIENT_HANDSHAKED,
    CLIENT_CONNECTED
} client_status_t;

typedef void (*dispatcher_callback) (uint8_t opcode, byte_pack_t* payload);

client_status_t get_client_status();
dkvr_err init_dkvr_client(dispatcher_callback callback);
dkvr_err update_client_connection();
dkvr_err dispatch_received_inst();
void send_client_inst(uint8_t opcode, uint8_t len, uint8_t align, const void* payload);

#ifdef __cplusplus
}
#endif