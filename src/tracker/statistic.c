#include "tracker/statistic.h"

#include "common/system_interface.h"
#include "common/dkvr_const.h"

#define BUFFER_SIZE     64

struct moving_window_accumulator
{
    uint32_t values[BUFFER_SIZE];
    uint32_t accumulator;
    int ptr;
};

static struct tracker_statistic tracker_statistic;
static struct moving_window_accumulator mwa_execution_time = {0};
static struct moving_window_accumulator mwa_interrupt_miss = {0};
static struct moving_window_accumulator mwa_imu_miss = {0};

static void accumulate(struct moving_window_accumulator* mwa, uint32_t value);


const struct tracker_statistic* tracker_statistic_get_struct_ptr() { return &tracker_statistic; }

void tracker_statistic_record_execution_time()
{
    static uint32_t last_record = 0;
    uint32_t now = dkvr_get_time();
    uint32_t timespan = now - last_record;
    last_record = now;

    accumulate(&mwa_execution_time, timespan);

    tracker_statistic.execution_time = (uint8_t)(mwa_execution_time.accumulator / BUFFER_SIZE);
}

void tracker_statistic_record_interrupt_miss()
{
    static uint32_t last_record = 0;
    uint32_t now = dkvr_get_time();
    uint32_t timespan = now - last_record;
    last_record = now;

    accumulate(&mwa_interrupt_miss, timespan);

    uint32_t miss_rate = DKVR_IMU_SAMPLING_RATE * BUFFER_SIZE * 100 / mwa_interrupt_miss.accumulator;
    tracker_statistic.interrupt_miss_rate = (uint8_t)miss_rate;
}

void tracker_statistic_record_imu_miss()
{
    static uint32_t last_record = 0;
    uint32_t now = dkvr_get_time();
    uint32_t timespan = now - last_record;
    last_record = now;

    accumulate(&mwa_imu_miss, timespan);
    
    uint32_t miss_rate = DKVR_IMU_SAMPLING_RATE * BUFFER_SIZE * 100 / mwa_imu_miss.accumulator;
    tracker_statistic.interrupt_miss_rate = (uint8_t)miss_rate;
}


static void accumulate(struct moving_window_accumulator* mwa, uint32_t value)
{
    mwa->accumulator += value;
    mwa->accumulator -= mwa->values[mwa->ptr];
    mwa->values[mwa->ptr++] = value;
    if (mwa->ptr == BUFFER_SIZE)
        mwa->ptr = 0;
}