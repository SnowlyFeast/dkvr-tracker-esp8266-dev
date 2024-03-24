#pragma once

#ifdef __cplusplus
extern "C" {
#endif

extern volatile int gpio_interrupt;

void init_tracker();
void update_tracker();

#ifdef __cplusplus
}
#endif