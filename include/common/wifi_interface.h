#pragma once

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

int dkvr_wifi_is_connected();

dkvr_err dkvr_wifi_init();
dkvr_err dkvr_wifi_connect(const char* ssid, const char* password);
void dkvr_wifi_update_conection();

#ifdef __cplusplus
}
#endif