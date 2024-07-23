#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct eskf_nominal_state
{
    float orientation[4];               // orientation quaternion (w, x, y, z order)
    float linear_accel[3];              // linear acceleration in gravity unit(g)
    float magnetic_dist[3];             // magnetic disturbance in normalized Gauss (NG)
};

struct eskf_error_state
{
    float orientation_error[3];         // orientation error
    float linear_accel_error[3];        // linear acceleration error
    float magnetic_dist_error[3];       // magnetic disturbance error
};

struct eskf_configuration
{
    float time_step;                    // time step : second
    float noise_gyro[3];                // gyroscope noise variance : (rad/s)^2
    float noise_accel[3];               // accelerometer noise variance : (m/s^2)^2
    float noise_mag[3];                 // magnetometer noise variance : (NG)^2
    float uncertainty_linear_accel;     // uncertainty of linear acceleration model : (m/s^2)^s
    float uncertainty_magnetic_dist;    // uncertainty of magnetic disturbance model : (NG)^2
    float lpf_cutoff_linear_accel;      // cutoff freq of linear acceleration lpf : Hz
    float lpf_cutoff_magnetic_dist;     // cutoff freq of magnetic disturbance lpf : Hz
};

struct eskf_vector3
{
    float data[3];
};

extern struct eskf_nominal_state eskf_nominal;
extern struct eskf_error_state eskf_error;

void eskf_configure(struct eskf_configuration *config);
void eskf_init(struct eskf_vector3 accel_read, struct eskf_vector3 mag_read);
void eskf_update(struct eskf_vector3 gyro_read, struct eskf_vector3 accel_read, struct eskf_vector3 mag_read);

#ifdef __cplusplus
}
#endif