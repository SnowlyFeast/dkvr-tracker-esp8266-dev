#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum led_mode_e
{
    LED_MODE_MANUAL,
    LED_MODE_SLOWEST,
    LED_MODE_NORMAL,
    LED_MODE_FASTEST,
    LED_MODE_DOUBLE_TAB,
    LED_MODE_TRIPLE_TAB
} led_mode_t;

led_mode_t get_led_mode();
void turn_on_led();
void turn_off_led();
void set_led_mode(led_mode_t mode);
void interrupt_led_for_locate();
void interrupt_led_for_low_battery();
void update_led();

#ifdef __cplusplus
}
#endif