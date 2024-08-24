#include "led/led_control.h"

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/gpio_interface.h"
#include "common/system_interface.h"

#define LED_LOCATE_DURATION         2900UL
#define LED_LOW_BATTERY_DURATION    950UL

struct led_signal_pattern
{
    uint16_t cycle_period;
    uint16_t active_time;
    uint16_t repeat;
    uint16_t delay;
};

static const struct led_signal_pattern pattern_slow PROGMEM = {3000, 1500, 1, 0};
static const struct led_signal_pattern pattern_norm PROGMEM = {1500, 750, 1, 0};
static const struct led_signal_pattern pattern_fast PROGMEM = {500, 250, 1, 0};
static const struct led_signal_pattern pattern_2tap PROGMEM = {250, 125, 2, 500};
static const struct led_signal_pattern pattern_3tap PROGMEM = {200, 100, 3, 400};

static dkvr_led_mode led_mode = DKVR_LED_MODE_MANUAL;
static int led_on = 0;

static struct led_signal_pattern pattern;
static uint32_t cycle_begin = 0;

static int interrupt_enabled = 0;
static uint32_t interrupt_end = 0;
static dkvr_led_mode memory_led_mode = DKVR_LED_MODE_MANUAL;
static int memory_led_on = 0;

static void set_interrupt(dkvr_led_mode mode, uint32_t duration);
static void clear_interrupt();


dkvr_led_mode dkvr_led_get_mode()
{
    return led_mode;
}

void dkvr_led_init()
{
    dkvr_gpio_mode(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_GPIO_MODE_OUTPUT);
}

void dkvr_led_on()
{
    if (led_on)
        return;

#ifdef DKVR_HARDWARE_LED_ACTIVE_LOW
    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_LOW);
#else
    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_HIGH);
#endif
    led_on = 1;
}

void dkvr_led_off()
{
    if (!led_on)
        return;

#ifdef DKVR_HARDWARE_LED_ACTIVE_LOW
    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_HIGH);
#else
    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_LOW);
#endif
    led_on = 0;
}

void dkvr_led_set_mode(dkvr_led_mode mode)
{
    // save to memory if interrupt is ongoing
    if (interrupt_enabled)
    {
        memory_led_mode = mode;
        return;
    }

    // same mode
    if (led_mode == mode)
        return;

    switch (mode)
    {
    default:
    case DKVR_LED_MODE_MANUAL:
        break;

    case DKVR_LED_MODE_SLOWEST:
        memcpy_P(&pattern, &pattern_slow, sizeof(struct led_signal_pattern));
        break;
        
    case DKVR_LED_MODE_NORMAL:
        memcpy_P(&pattern, &pattern_norm, sizeof(struct led_signal_pattern));
        break;
        
    case DKVR_LED_MODE_FASTEST:
        memcpy_P(&pattern, &pattern_fast, sizeof(struct led_signal_pattern));
        break;
        
    case DKVR_LED_MODE_DOUBLE_TAP:
        memcpy_P(&pattern, &pattern_2tap, sizeof(struct led_signal_pattern));
        break;
        
    case DKVR_LED_MODE_TRIPLE_TAP:
        memcpy_P(&pattern, &pattern_3tap, sizeof(struct led_signal_pattern));
        break;
    }

    led_mode = mode;
    cycle_begin = dkvr_get_time();
}

void dkvr_led_interrupt_for_locate()
{
    if (!interrupt_enabled)
        set_interrupt(DKVR_LED_MODE_DOUBLE_TAP, LED_LOCATE_DURATION);
}

void dkvr_led_interrupt_for_low_battery()
{
    if (!interrupt_enabled)
        set_interrupt(DKVR_LED_MODE_TRIPLE_TAP, LED_LOW_BATTERY_DURATION);
}

void dkvr_led_update()
{
    uint32_t now = dkvr_get_time();

    // check interrupt state
    if (interrupt_enabled && (now >= interrupt_end))
        clear_interrupt();

    // nothing to update on manual mode
    if (led_mode == DKVR_LED_MODE_MANUAL)
        return;

    // calculate timetick position
    uint32_t elapsed = now - cycle_begin;
    for (int i = 0; i < pattern.repeat; i++)
    {
        if (elapsed < pattern.cycle_period)
        {
            // timetick is on cycle preriod
            (elapsed < pattern.active_time) ? dkvr_led_on() : dkvr_led_off();
            return;
        }
        else
        {
            // timetick has passed the cycle
            elapsed -= pattern.cycle_period;
        }
    }

    if (elapsed < pattern.delay)
    {
        // timetick is on delay period
        dkvr_led_off();
        return;
    }
    else
    {
        // full pattern ended, start new pattern
        elapsed -= pattern.delay;
        cycle_begin = dkvr_get_time() - elapsed;
        (elapsed < pattern.active_time) ? dkvr_led_on() : dkvr_led_off();
    }
}

static void set_interrupt(dkvr_led_mode mode, uint32_t duration)
{
    memory_led_mode = led_mode;
    memory_led_on = led_on;
    dkvr_led_set_mode(mode);
    interrupt_enabled = 1;
    interrupt_end = dkvr_get_time() + duration;
}

static void clear_interrupt()
{
    interrupt_enabled = 0;
    dkvr_led_set_mode(memory_led_mode);
    if (led_mode == DKVR_LED_MODE_MANUAL)
        memory_led_on ? dkvr_led_on() : dkvr_led_off();
}