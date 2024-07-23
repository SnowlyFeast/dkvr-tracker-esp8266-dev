#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

// the flag clears to 0 after the value has been read
int is_tracker_err_raised();

void update_tracker_status();

uint8_t get_tracker_status_size();
void* get_tracker_status_ptr();

dkvr_err get_tracker_init_result();
dkvr_err get_tracker_last_err();
uint8_t get_tracker_battery_perc();

void set_tracker_init_result(dkvr_err result);
void set_tracker_last_err(dkvr_err err);

#ifdef __cplusplus
}
#endif