#include "tracker/behavior.h"


#define BEHAVIOR_ACTIVE_BIT     0b00000001
#define BHEAVIOR_RAW_BIT        0b00000010
#define BEHAVIOR_LED_BIT        0b00000100

static int updated = 0;
static uint8_t behavior;

int tracker_behavior_is_updated()
{
    int temp = updated;
    updated = 0;
    return temp;
}

int tracker_behavior_get_active() { return behavior & BEHAVIOR_ACTIVE_BIT; }
int tracker_behavior_get_raw() { return behavior & BHEAVIOR_RAW_BIT; }
int tracker_behavior_get_led() { return behavior & BEHAVIOR_LED_BIT; }

void tracker_behavior_reset()
{
    behavior = BEHAVIOR_LED_BIT;
    updated = 1;
}

void tracker_behavior_set(uint8_t new_behavior)
{
    behavior = new_behavior;
    updated = 1;
}