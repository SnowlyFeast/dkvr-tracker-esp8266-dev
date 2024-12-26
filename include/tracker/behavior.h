#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int tracker_behavior_is_updated();
int tracker_behavior_get_led();
int tracker_behavior_get_active();
int tracker_behavior_get_raw();
int tracker_behavior_get_nominal();

void tracker_behavior_reset();
void tracker_behavior_set(uint8_t new_behavior);

#ifdef __cplusplus
}
#endif