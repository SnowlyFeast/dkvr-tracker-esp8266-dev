#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"
#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void tracker_instruction_handler(uint8_t opcode, union dkvr_byte_pack* payload);

#ifdef __cplusplus
}
#endif