#include "tracker/tracker_status.h"

#include "math.h"

#include "common/dkvr_const.h"
#include "common/adc_interface.h"
#include "common/system_interface.h"

#define ADC_BAT_CUTOFF_VAL          ((uint16_t) (DKVR_BAT_CUTOFF_VOLTAGE  * ESP8266_ADC_RESOLUTION / 5000 + 1))
#define ADC_BAT_MAXIMUM_VAL         ((uint16_t) (DKVR_BAT_MAXIMUM_VOLTAGE * ESP8266_ADC_RESOLUTION / 5000    ))
#define ADC_BAT_EFFECTIVE_RANGE     (ADC_BAT_MAXIMUM_VAL - ADC_BAT_CUTOFF_VAL)

typedef struct tracker_status_s
{
    dkvr_err init_result;
    dkvr_err last_err;
    uint8_t battery_perc;
} tracker_status_t;

static tracker_status_t tracker_status = {};
static int err_raised = 0;

static void update_battery_perc();

int is_tracker_err_raised()
{
    int temp = err_raised;
    err_raised = 0;
    return temp;
}

void update_tracker_status()
{
    // update battery perc
    static uint32_t last_battery_read = 0;
    if (dkvr_approximate_time > last_battery_read + DKVR_BAT_READ_INTERVAL)
    {
        last_battery_read = dkvr_approximate_time;
        update_battery_perc();
    }
}

uint8_t get_tracker_status_size()
{
    return sizeof(tracker_status);
}

void* get_tracker_status_ptr()
{
    return &tracker_status;
}

dkvr_err get_tracker_init_result()
{
    return tracker_status.init_result;
}

dkvr_err get_tracker_last_err()
{
    return tracker_status.last_err;
}

uint8_t get_tracker_battery_perc()
{
    return tracker_status.battery_perc;
}

void set_tracker_init_result(dkvr_err result)
{
    tracker_status.init_result = result;
}

void set_tracker_last_err(dkvr_err err)
{
    tracker_status.last_err = err;
    err_raised = 1;
}

static void update_battery_perc()
{
    uint16_t adc_value = dkvr_adc_read();
    uint8_t perc = (uint8_t)((adc_value - ADC_BAT_CUTOFF_VAL) * (100.0f / ADC_BAT_EFFECTIVE_RANGE));

    tracker_status.battery_perc = perc;
}