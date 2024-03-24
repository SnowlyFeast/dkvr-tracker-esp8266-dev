#include "common/esp8266_util.h"

#include <Arduino.h>
#include <user_interface.h>

extern os_timer_t* timer_list;

static uint32_t int_wakeup_gpio = 14;

void esp_set_int_wakeup_gpio(uint32_t gpio_num)
{
    if (gpio_num == 16)
        return;
    int_wakeup_gpio = gpio_num;
}

void esp_light_sleep(uint32_t millis, int wake_on_int, wakeup_callback callback)
{
    // disconnect os-timer
    timer_list = nullptr;

    // enable light sleep
    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();

    // enable interrupt wakeup
    if (wake_on_int)
        gpio_pin_wakeup_enable(int_wakeup_gpio, GPIO_PIN_INTR_HILEVEL);

    // attach callback
    if (callback)
        wifi_fpm_set_wakeup_cb(callback);

    wifi_fpm_do_sleep(millis * 1000);
    delay(millis + 1);
}

void esp_deep_sleep(uint32_t millis)
{
    system_deep_sleep_set_option(static_cast<int>(RF_DEFAULT));
    system_deep_sleep(static_cast<uint64_t>(millis) * 1000);
    esp_suspend();
}

uint32_t rtc_millis()
{
    return (system_get_rtc_time() * (system_rtc_clock_cali_proc() >> 12) / 1000);
}