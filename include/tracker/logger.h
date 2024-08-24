#pragma once

#include "common/dkvr_core.h"

#define DKVR_LOGGER_MSG_LEN         (DKVR_NET_UDP_PAYLOAD_SIZE - 5)

#ifdef __cplusplus
extern "C" {
#endif

struct dkvr_error_log
{
    uint32_t timestamp;
    dkvr_err err;
    char msg[DKVR_LOGGER_MSG_LEN];
};

const struct dkvr_error_log* tracker_logger_get_list();
int tracker_logger_get_count();
void tracker_logger_flush();
void tracker_logger_push(dkvr_err err, PGM_P msg);

#ifdef __cplusplus
}
#endif