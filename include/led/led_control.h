#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DKVR_LED_MODE_MANUAL,
    DKVR_LED_MODE_SLOWEST,
    DKVR_LED_MODE_NORMAL,
    DKVR_LED_MODE_FASTEST,
    DKVR_LED_MODE_DOUBLE_TAP,
    DKVR_LED_MODE_TRIPLE_TAP
} dkvr_led_mode;

dkvr_led_mode dkvr_led_get_mode();

void dkvr_led_on();
void dkvr_led_off();
void dkvr_led_set_mode(dkvr_led_mode mode);
void dkvr_led_interrupt_for_locate();
void dkvr_led_interrupt_for_low_battery();
void dkvr_led_update();

#ifdef __cplusplus
}
#endif