#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*dkvr_instruction_handler) (uint8_t opcode, union dkvr_byte_pack* payload);

int dkvr_client_is_connected();

dkvr_err dkvr_client_init(dkvr_instruction_handler callback);
dkvr_err dkvr_client_update_connection();
dkvr_err dkvr_client_dispatch_received();
dkvr_err dkvr_client_send_instruction(uint8_t opcode, uint8_t len, uint8_t align, const void* payload);

#ifdef __cplusplus
}
#endif