#include "led/led_control.h"

#include "common/dkvr_const.h"
#include "common/dkvr_core.h"
#include "common/gpio_interface.h"
#include "common/system_interface.h"

#define LED_SLOWEST_CYCLE_PERIOD    3000
#define LED_SLOWEST_ACTIVE_TIME     1500
#define LED_SLOWEST_REPEAT          1
#define LED_SLOWEST_DELAY           0

#define LED_NORMAL_CYCLE_PERIOD     2000
#define LED_NORMAL_ACTIVE_TIME      1000
#define LED_NORMAL_REPEAT           1
#define LED_NORMAL_DELAY            0

#define LED_FASTEST_CYCLE_PERIOD    500
#define LED_FASTEST_ACTIVE_TIME     250
#define LED_FASTEST_REPEAT          1
#define LED_FASTEST_DELAY           0

#define LED_DOUBLETAB_CYCLE_PERIOD  250
#define LED_DOUBLETAB_ACTIVE_TIME   125
#define LED_DOUBLETAB_REPEAT        2
#define LED_DOUBLETAB_DELAY         500

#define LED_TRIPLETAB_CYCLE_PERIOD  200
#define LED_TRIPLETAB_ACTIVE_TIME   100
#define LED_TRIPLETAB_REPEAT        3
#define LED_TRIPLETAB_DELAY         400

#define LED_LOCATE_DURATION         2900UL
#define LED_LOW_BATTERY_DURATION    950UL

static led_mode_t current_mode = LED_MODE_MANUAL;
static int current_led_on = 0;

static uint32_t cycle_begin = 0;
static uint16_t cycle_period = 0;
static uint16_t active_time = 0;
static uint16_t repeat = 0;
static uint16_t delay = 0;

static int interrupt_enabled = 0;
static uint32_t interrupt_begin = 0;
static uint32_t interrupt_duration = 0;
static led_mode_t memory_mode = LED_MODE_MANUAL;
static int memory_led_on = 0;

static void enable_interrupt(led_mode_t mode, uint32_t duration);
static void disable_interrupt();

led_mode_t get_led_mode()
{
    return current_mode;
}

void turn_on_led()
{
    if (current_led_on)
        return;

    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_HIGH);
    current_led_on = 1;
}

void turn_off_led()
{
    if (!current_led_on)
        return;

    dkvr_gpio_write(DKVR_HARDWARE_LED_GPIO_NUM, DKVR_LOW);
    current_led_on = 0;
}

void set_led_mode(led_mode_t mode)
{
    if (interrupt_enabled)
    {
        memory_mode = mode;
        return;
    }

    if (current_mode == mode)
        return;

    switch (mode)
    {
    default:
    case LED_MODE_MANUAL:
        break;

    case LED_MODE_SLOWEST:
        cycle_period = LED_SLOWEST_CYCLE_PERIOD;
        active_time = LED_SLOWEST_ACTIVE_TIME;
        repeat = LED_SLOWEST_REPEAT;
        delay = LED_SLOWEST_DELAY;
        break;

    case LED_MODE_NORMAL:
        cycle_period = LED_NORMAL_CYCLE_PERIOD;
        active_time = LED_NORMAL_ACTIVE_TIME;
        repeat = LED_NORMAL_REPEAT;
        delay = LED_NORMAL_DELAY;
        break;

    case LED_MODE_FASTEST:
        cycle_period = LED_FASTEST_CYCLE_PERIOD;
        active_time = LED_FASTEST_ACTIVE_TIME;
        repeat = LED_FASTEST_REPEAT;
        delay = LED_FASTEST_DELAY;
        break;

    case LED_MODE_DOUBLE_TAB:
        cycle_period = LED_DOUBLETAB_CYCLE_PERIOD;
        active_time = LED_DOUBLETAB_ACTIVE_TIME;
        repeat = LED_DOUBLETAB_REPEAT;
        delay = LED_DOUBLETAB_DELAY;
        break;

    case LED_MODE_TRIPLE_TAB:
        cycle_period = LED_TRIPLETAB_CYCLE_PERIOD;
        active_time = LED_TRIPLETAB_ACTIVE_TIME;
        repeat = LED_TRIPLETAB_REPEAT;
        delay = LED_TRIPLETAB_DELAY;
        break;
    }

    current_mode = mode;
    cycle_begin = dkvr_get_time();
}

void interrupt_led_for_locate()
{
    if (!interrupt_enabled)
    {
        enable_interrupt(LED_MODE_DOUBLE_TAB, LED_LOCATE_DURATION);
    }
}

void interrupt_led_for_low_battery()
{
    if (!interrupt_enabled)
    {
        enable_interrupt(LED_MODE_TRIPLE_TAB, LED_LOW_BATTERY_DURATION);
    }
}

void update_led()
{
    if (interrupt_enabled)
    {
        if (dkvr_get_time() > (interrupt_begin + interrupt_duration))
        {
            disable_interrupt();
        }
    }
    
    if (current_mode == LED_MODE_MANUAL)
        return;

    uint32_t elapsed = dkvr_get_time() - cycle_begin;

    for (int i = 0; i < repeat; i++)
    {
        if (elapsed < cycle_period)
        {
            // timetick on cycle period
            elapsed < active_time ? turn_on_led() : turn_off_led();
            return;
        }
        else
        {
            // next cycle
            elapsed -= cycle_period;
        }
    }

    if (elapsed < delay)
    {
        // timetick on delay period
        turn_off_led();
        return;
    }
    else
    {
        // full-cycle ended
        elapsed -= delay;
        cycle_begin = dkvr_get_time() - elapsed;
        elapsed < active_time ? turn_on_led() : turn_off_led();
    }
}

static void enable_interrupt(led_mode_t mode, uint32_t duration)
{
    memory_mode = current_mode;
    memory_led_on = current_led_on;
    set_led_mode(mode);
    interrupt_enabled = 1;
    interrupt_begin = dkvr_get_time();
    interrupt_duration = duration;
}

static void disable_interrupt()
{
    interrupt_enabled = 0;
    set_led_mode(memory_mode);
    if (current_mode == LED_MODE_MANUAL)
    {
        memory_led_on ? turn_on_led() : turn_off_led();
    }
}