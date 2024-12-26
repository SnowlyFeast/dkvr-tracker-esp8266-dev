#include "tracker/statistic.h"

#include "common/system_interface.h"
#include "common/dkvr_const.h"

#include "common/dkvr_core.h"

// BUFFER_SIZE should not larger than 64
#define BUFFER_SIZE     64

static struct tracker_statistic statistic;

static int ptr = 0;
static uint32_t exec_accumulator = 0;
static int int_miss_accumulator = 0;
static int imu_miss_accumulator = 0;

static uint32_t exec_values[BUFFER_SIZE] = {0};
static uint64_t int_miss_values = 0;
static uint64_t imu_miss_values = 0;

const struct tracker_statistic* tracker_statistic_get_struct_ptr() { return &statistic; }

void tracker_statistic_record_cycle_end()
{
    // record execution time
    static uint32_t last_record = 0;
    uint32_t now = dkvr_get_time();
    uint32_t cycle_time = now - last_record;
    last_record = now;
    exec_accumulator += cycle_time;
    exec_accumulator -= exec_values[ptr];
    exec_values[ptr] = cycle_time;
    statistic.execution_time = (uint8_t)(exec_accumulator / BUFFER_SIZE);

    // update pointer
    if (++ptr == BUFFER_SIZE)
        ptr = 0;

    // update interrupt miss and imu miss
    uint64_t bitmask = 0x1u << ptr;
    if (int_miss_values & bitmask)
    {
        int_miss_accumulator--;
        int_miss_values &= ~bitmask;
    }
    if (imu_miss_values & bitmask)
    {
        imu_miss_accumulator--;
        imu_miss_values &= ~bitmask;
    }
}

void tracker_statistic_record_interrupt_miss()
{
    static int last_ptr = -1;
    if (ptr == last_ptr) return;
    last_ptr = ptr;

    int_miss_accumulator++;
    int_miss_values |= (0x1 << ptr);

    statistic.interrupt_miss_rate = (uint8_t)(int_miss_accumulator * 100 / BUFFER_SIZE);
}

void tracker_statistic_record_imu_miss()
{
    static int last_ptr = -1;
    if (ptr == last_ptr) return;
    last_ptr = ptr;

    imu_miss_accumulator++;
    imu_miss_values |= (0x1 << ptr);

    statistic.imu_miss_rate = (uint8_t)(imu_miss_accumulator * 100 / BUFFER_SIZE);
}
