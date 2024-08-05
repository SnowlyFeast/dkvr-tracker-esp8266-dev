#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// FIXME
typedef void (*wakeup_callback) (void);

void esp_set_int_wakeup_gpio(uint32_t gpio_num);
void esp_light_sleep(uint32_t millis, int wake_on_int, wakeup_callback callback);
void esp_deep_sleep(uint32_t millis);
uint32_t rtc_millis();

#ifdef __cplusplus
}
#endif