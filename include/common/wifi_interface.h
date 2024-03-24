#pragma once

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum wifi_status_e
{
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_CONNECTED_TO_AP
} wifi_status_t;


wifi_status_t get_wifi_status();

dkvr_err_t dkvr_wifi_init();
dkvr_err_t dkvr_wifi_connect(const char* ssid, const char* password);
int is_wifi_status_changed();

#ifdef __cplusplus
}
#endif