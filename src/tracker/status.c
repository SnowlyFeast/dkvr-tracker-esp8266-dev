#include "tracker/status.h"

#include <math.h>

#include "common/dkvr_const.h"
#include "common/adc_interface.h"
#include "common/system_interface.h"


static struct tracker_status status = {0};

static void update_battery_level();


const struct tracker_status* tracker_status_get_struct_ptr() { return &status; }

uint8_t tracker_status_get_battery_level() { return status.battery_level; }

void tracker_status_update()
{
    update_battery_level();
}

void tracker_status_set_init_result(dkvr_err result)
{
    status.init_result = result;
}


static void update_battery_level()
{
    static uint32_t last_battery_read = 0;
    if (dkvr_approximate_time > last_battery_read + DKVR_BAT_READ_INTERVAL)
    {
        uint16_t adc = dkvr_adc_read();
        uint8_t perc = (uint8_t)((adc - DKVR_BAT_CUTOFF_VAL) * (100.0f / DKVR_BAT_EFFECTIVE_RANGE));

        status.battery_level = perc;
        last_battery_read = dkvr_approximate_time;
    }
}