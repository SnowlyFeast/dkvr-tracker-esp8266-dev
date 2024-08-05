#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tracker_main_set_gpio_interrupted();

int tracker_main_init();
void tracker_main_update();

#ifdef __cplusplus
}
#endif