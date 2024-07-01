#include "tracker/tracker_statistic.h"

#include "common/system_interface.h"
#include "common/dkvr_core.h"

#define BUFFER_BIT_SHIFT    6
#define BUFFER_SIZE         (1 << BUFFER_BIT_SHIFT)

static tracker_statistic_t tracker_statistic = {};
static uint32_t execution_times[BUFFER_SIZE] = {};
static int ptr = 0;
static uint32_t accumulator = 0;

tracker_statistic_t get_tracker_statistic()
{
    return tracker_statistic;
}

void record_execution_time()
{
    static uint32_t last_record = 0;

    uint32_t now = dkvr_get_time();
    uint32_t timespan = now - last_record;
    last_record = now;

    accumulator += timespan;
    accumulator -= execution_times[ptr];
    execution_times[ptr++] = timespan;
    if (ptr == BUFFER_SIZE)
        ptr = 0;

    tracker_statistic.execution_time = (uint8_t)(accumulator / BUFFER_SIZE);
}