#pragma once

#include <stdint.h>

#include "common/dkvr_const.h"

#ifdef __cplusplus
extern "C" {
#endif

struct tracker_status
{
    dkvr_err init_result;
    uint8_t battery_level;
};

const struct tracker_status* tracker_status_get_struct_ptr();

uint8_t tracker_status_get_battery_level();
void tracker_status_update();
void tracker_status_set_init_result(dkvr_err result);

#ifdef __cplusplus
}
#endif