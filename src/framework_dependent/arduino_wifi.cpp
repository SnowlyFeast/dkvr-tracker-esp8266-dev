#include "common/wifi_interface.h"

#include <ESP8266WiFi.h>

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"

static int wifi_connected = 0;

int dkvr_wifi_is_connected()
{
    return wifi_connected;
}

dkvr_err dkvr_wifi_init()
{
    return DKVR_OK;
}

dkvr_err dkvr_wifi_connect(const char* ssid, const char* password)
{
    if (wifi_connected)
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
    wifi_connected = 1;
    
    return DKVR_OK;
}

void dkvr_wifi_update_conection()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (wifi_connected)
        {
            wifi_connected = 0;
            WiFi.disconnect();
        }
    }
}