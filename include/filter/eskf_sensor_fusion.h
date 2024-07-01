#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct eskf_nominal_state
{
    float orientation[4];
    float angular_rate[3];
    float gravity[3];
    float earth_magnetic[3];
    float angular_bias[3];
    float linear_accel[3];
    float magnetic_dist[3];
};

struct eskf_error_state
{
    float delta_ori[3];
    float delta_bias[3];
    float delta_accel[3];
    float delta_dist[3];
};

struct eskf_vector3
{
    float data[3];
};

extern struct eskf_nominal_state eskf_nominal;
extern struct eskf_error_state eskf_error;

void eskf_configure(float time_step, float decay_factor_accel, float decay_factor_mag,
                    float noise_gyro, float noise_accel, float noise_mag,
                    float linear_accel, float magnetic_dist);
void eskf_init(struct eskf_vector3 accel_read, struct eskf_vector3 mag_read);
void eskf_update(struct eskf_vector3 gyro_read, struct eskf_vector3 accel_read, struct eskf_vector3 mag_read);

#ifdef __cplusplus
}
#endif