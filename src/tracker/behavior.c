#include "tracker/behavior.h"

#include "common/dkvr_core.h"

#define BEHAVIOR_LED_BIT        0b00000001
#define BEHAVIOR_ACTIVE_BIT     0b00000010
#define BHEAVIOR_RAW_BIT        0b00000100
#define BEHAVIOR_NOMINAL_BIT    0b00001000

static int updated = 0;

struct Behavior
{
    int led;
    int active;
    int raw;
    int nominal;
} behavior = {0};

int tracker_behavior_is_updated()
{
    int temp = updated;
    updated = 0;
    return temp;
}

int tracker_behavior_get_led()      { return behavior.led; }
int tracker_behavior_get_active()   { return behavior.active; }
int tracker_behavior_get_raw()      { return behavior.raw; }
int tracker_behavior_get_nominal()  { return behavior.nominal; }

void tracker_behavior_reset()
{
    static const struct Behavior default_behavior PROGMEM = 
    {
        .led = 1,
        .active = 0,
        .raw = 0,
        .nominal = 0
    };

    memcpy_P(&behavior, &default_behavior, sizeof(behavior));
}

void tracker_behavior_set(uint8_t new_behavior)
{
    behavior.led     = new_behavior & BEHAVIOR_LED_BIT;
    behavior.active  = new_behavior & BEHAVIOR_ACTIVE_BIT;
    behavior.raw     = new_behavior & BHEAVIOR_RAW_BIT;
    behavior.nominal = new_behavior & BEHAVIOR_NOMINAL_BIT;
    updated = 1;
}