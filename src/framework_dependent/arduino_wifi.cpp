#include "common/wifi_interface.h"

#include <ESP8266WiFi.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"

static wifi_status_t wifi_status = WIFI_STATUS_DISCONNECTED;

wifi_status_t get_wifi_status()
{
    return wifi_status;
}

dkvr_err_t dkvr_wifi_init()
{
    return DKVR_OK;
}

dkvr_err_t dkvr_wifi_connect(const char* ssid, const char* password)
{
    if (wifi_status == WIFI_STATUS_CONNECTED_TO_AP)
        return DKVR_OK;
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    switch (WiFi.waitForConnectResult())
    {
    case WL_CONNECTED:
        break;
    case WL_NO_SSID_AVAIL:
        return DKVR_ERR_WIFI_NO_SUCH_SSID;
    case WL_WRONG_PASSWORD:
        return DKVR_ERR_WIFI_WRONG_PSWD;
    default:
        return DKVR_ERR_WIFI_UNKNOWN_ERR;
    }

    PRINTLN("WiFi connected with IP : ", WiFi.localIP().toString().c_str());
    wifi_status = WIFI_STATUS_CONNECTED_TO_AP;
    
    return DKVR_OK;
}

int is_wifi_status_changed()
{
    if (wifi_status == WIFI_STATUS_CONNECTED_TO_AP) {
        if (WiFi.status() != WL_CONNECTED) {
            wifi_status = WIFI_STATUS_DISCONNECTED;
            WiFi.disconnect();
            return 1;
        }
    }
    return 0;
}