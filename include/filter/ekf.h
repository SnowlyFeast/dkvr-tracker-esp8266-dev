#pragma once

#include "common/dkvr_types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float prio_state[4];
extern float post_state[4];

void ekf_set_nsd(float g_nsd, float a_nsd, float m_nsd);
void ekf_init(const vector3_t* accel, const vector3_t* mag);
void ekf_predict(const vector3_t *gyro, float dt);
void ekf_correct(const vector3_t *accel, const vector3_t *mag, quaternion_t *out);

#ifdef __cplusplus
}
#endif