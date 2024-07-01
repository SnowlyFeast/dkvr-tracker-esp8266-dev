#include "filter/eskf_sensor_fusion.h"

#include <math.h>
#include <string.h>

#include "filter/matrix.h"

// state variable
int eskf_ready = 0;
struct eskf_nominal_state eskf_nominal;
struct eskf_error_state eskf_error;

// computation variable
static float gravity_ref[3] = { 0.0f, 0.0f, -1.0f };
static float magnetic_ref[3];
static float kalman_gain[12][6];

// configuration variable
static float time_step = 0.01f;                     // sec
static float decay_factor_accel = 0.5f;             // 0 ~ 1.0
static float decay_factor_mag = 0.5f;               // 0 ~ 1.0
static float noise_gyro = 7.6e-7f;                  // ( rad/s )^2
static float noise_accel = 1.6e-5f;                 // ( m/s^2 )^2
static float noise_mag = 4.0e-6f;                   // ( G )^2
static float uncertainty_linear_accel = 3.8e-5f;    // ( m/s^2 )^2
static float uncertainty_magnetic_dist = 2.0e-5f;   // ( G )^2

// protos
static void update_nominal_state(float* gyro_read);
static void compute_kalman_gain();
static void compute_posteriori_error_state(float* accel_read, float* mag_read);
static void inject_error_state();

static void normalize_vec(float* vec);
static void normalize_quat(float* quat);
static void euler_rodrigues(const float* quat, const float* vec, float* dst);
static void hamilton_product2(const float* lhs, const float w, const float* v, float* dst);
static void small_angle_rotation_matrix(const float* vec, float (*dst)[3]);


void eskf_configure(float ts, float dfa, float dfm, float ng, float na, float nm, float ula, float umd)
{
    time_step = ts;
    decay_factor_accel = dfa;
    decay_factor_mag = dfm;
    noise_gyro = ng;
    noise_accel = na;
    noise_mag = nm;
    uncertainty_linear_accel = ula;
    uncertainty_magnetic_dist = umd;
}

void eskf_init(struct eskf_vector3 accel_read, struct eskf_vector3 mag_read)
{
    // normalize
    normalize_vec(accel_read.data);
    normalize_vec(mag_read.data);

    // calculate mag ref
    float cos = sqrtf(0.5f - 0.5f * accel_read.data[2]);
    float sin = sqrtf(0.5f + 0.5f * accel_read.data[2]);
    float size = sqrtf(accel_read.data[0] * accel_read.data[0] + accel_read.data[1] * accel_read.data[1]);
    float quat[4] = { cos, -accel_read.data[1] * sin / size, accel_read.data[0] * sin / size, 0 };
    euler_rodrigues(quat, mag_read.data, magnetic_ref);

    // remove y component (remove magnetic declination)
    magnetic_ref[0] = sqrtf(magnetic_ref[0] * magnetic_ref[0] + magnetic_ref[1] * magnetic_ref[1]);
    magnetic_ref[1] = 0;

    // find rotation matrix (TRIAD method)
    float mat1[9], mat2[9];
    float temp1[3], temp2[3];
    matrix_vector_cross(gravity_ref, magnetic_ref, temp1);
    matrix_vector_cross(gravity_ref, temp1, temp2);
    normalize_vec(temp1);
    normalize_vec(temp2);
    memcpy(mat1 + 0, gravity_ref, sizeof(float) * 3);
    memcpy(mat1 + 3, temp1, sizeof(float) * 3);
    memcpy(mat1 + 6, temp2, sizeof(float) * 3);

    matrix_vector_cross(accel_read.data, mag_read.data, temp1);
    matrix_vector_cross(accel_read.data, temp1, temp2);
    normalize_vec(temp1);
    normalize_vec(temp2);
    memcpy(mat2 + 0, accel_read.data, sizeof(float) * 3);
    memcpy(mat2 + 3, temp1, sizeof(float) * 3);
    memcpy(mat2 + 6, temp2, sizeof(float) * 3);

    float rot_mat[3][3];
    matrix_mul_trans(3, 3, 3, mat2, mat1, FLAT(rot_mat));

    // calculate initial orientation
    quat[0] = 0.5f * sqrt(rot_mat[0][0] + rot_mat[1][1] + rot_mat[2][2] + 1);
    float fqw = 4.0f * quat[0];
    quat[1] = (rot_mat[2][1] - rot_mat[1][2]) / fqw;
    quat[2] = (rot_mat[0][2] - rot_mat[2][0]) / fqw;
    quat[3] = (rot_mat[1][0] - rot_mat[0][1]) / fqw;
    
    // orientation
    memcpy(eskf_nominal.orientation, quat, sizeof(quat));
    
    // gravity and earth magnetic
    quat[1] *= -1;
    quat[2] *= -1;
    quat[3] *= -1;
    euler_rodrigues(quat, gravity_ref, eskf_nominal.gravity);
    euler_rodrigues(quat, magnetic_ref, eskf_nominal.earth_magnetic);

    // angular rate, bias, linear acceleration and magnetic disturbance
    float zero[3] = { 0.0f };
    memcpy(eskf_nominal.angular_rate, zero, sizeof(zero));
    memcpy(eskf_nominal.angular_bias, zero, sizeof(zero));
    memcpy(eskf_nominal.linear_accel, zero, sizeof(zero));
    memcpy(eskf_nominal.magnetic_dist, zero, sizeof(zero));

    // reset error state
    memcpy(eskf_error.delta_ori, zero, sizeof(zero));
    memcpy(eskf_error.delta_bias, zero, sizeof(zero));
    memcpy(eskf_error.delta_accel, zero, sizeof(zero));
    memcpy(eskf_error.delta_dist, zero, sizeof(zero));
    
    eskf_ready = 1;
}

void eskf_update(struct eskf_vector3 gyro_read, struct eskf_vector3 accel_read, struct eskf_vector3 mag_read)
{
    if (!eskf_ready) return;
    update_nominal_state(gyro_read.data);
    compute_kalman_gain();
    compute_posteriori_error_state(accel_read.data, mag_read.data);
    inject_error_state();
}

static void update_nominal_state(float* gyro_read)
{
    struct eskf_nominal_state updated;
    float temp_vec[3];
    float temp_mat[3][3];

    // angular rate
    for (int i = 0; i < 3; i++) {
        updated.angular_rate[i] = gyro_read[i] - eskf_nominal.angular_bias[i];
        temp_vec[i] = updated.angular_rate[i] * time_step * 0.5f;
    }

    // angular bias
    memcpy(updated.angular_bias, eskf_nominal.angular_bias, sizeof(eskf_nominal.angular_bias));

    // orientation
    hamilton_product2(eskf_nominal.orientation, 1.0f, temp_vec, updated.orientation);
    normalize_quat(updated.orientation);

    // gravity and earth magnetic
    matrix_mul_scalar(3, 1, temp_vec, -1.0f);
    small_angle_rotation_matrix(temp_vec, temp_mat);
    matrix_mul(3, 3, 1, FLAT(temp_mat), eskf_nominal.gravity, updated.gravity);
    matrix_mul(3, 3, 1, FLAT(temp_mat), eskf_nominal.earth_magnetic, updated.earth_magnetic);
    
    // linear acceleration and magnetic disturbance
    matrix_mul(3, 3, 1, FLAT(temp_mat), eskf_nominal.linear_accel, updated.linear_accel);
    matrix_mul(3, 3, 1, FLAT(temp_mat), eskf_nominal.magnetic_dist, updated.magnetic_dist);
    for (int i = 0; i < 3; i++)
    {
        updated.linear_accel[i] *= decay_factor_accel;
        updated.magnetic_dist[i] *= decay_factor_mag;
    }

    memcpy(&eskf_nominal, &updated, sizeof(struct eskf_nominal_state));
}

static void compute_kalman_gain()
{
    float temp_vec[3];
    float temp_mat[3][3];

    // // compute observation matrix component (Ω_a and Ω_m)
    // float omega_a[3][3] = {0};
    // float omega_m[3][3] = {0};
    // for (int i = 0; i < 3; i++)
    //     temp_vec[i] = eskf_nominal.angular_rate[i] * time_step / 2;
    // small_angle_rotation_matrix(temp_vec, temp_mat);

    // matrix_sub(3, 1, eskf_nominal.gravity, eskf_nominal.linear_accel, temp_vec);
    // matrix_mul(3, 3, 1, FLAT(temp_mat), temp_vec, FLAT(omega_a));
    // matrix_skew_symmetrize(FLAT(omega_a), FLAT(omega_a));

    // matrix_add(3, 1, eskf_nominal.earth_magnetic, eskf_nominal.magnetic_dist, temp_vec);
    // matrix_mul(3, 3, 1, FLAT(temp_mat), temp_vec, FLAT(omega_m));
    // matrix_skew_symmetrize(FLAT(omega_m), FLAT(omega_m));

    // compute observation matrix component (Ω_g and Ω_m)
    float omega_g[3][3] = {0};
    float omega_m[3][3] = {0};

    euler_rodrigues(eskf_nominal.orientation, gravity_ref, temp_vec);
    matrix_skew_symmetrize(temp_vec, FLAT(omega_g));

    euler_rodrigues(eskf_nominal.orientation, magnetic_ref, temp_vec);
    matrix_skew_symmetrize(temp_vec, FLAT(omega_m));

    // compute outside of inversion (-PH^T)
    float pht[12][6] = {0};
    memcpy(temp_mat, omega_g, sizeof(omega_g));
    matrix_mul_scalar(3, 3, FLAT(temp_mat), time_step);
    matrix_sub(3, 3, FLAT(temp_mat), FLAT(omega_m), FLAT(temp_mat));
    matrix_mul_scalar(3, 3, FLAT(temp_mat), -noise_gyro);

    for (int i = 0; i < 3; i++)
    {
        memcpy(pht[i], temp_mat, sizeof(float) * 3);
        memcpy(pht[i] + 3, temp_mat, sizeof(float) * 3);
        memcpy(pht[i + 3], temp_mat, sizeof(float) * 3);
        memcpy(pht[i + 3] + 3, temp_mat, sizeof(float) * 3);

        pht[i + 6][i] = uncertainty_linear_accel;
        pht[i + 9][i + 3] = -uncertainty_magnetic_dist;
    }

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            if (i == j) continue;
            pht[i][j] *= -time_step;
            pht[i + 3][j + 3] *= -time_step;
            pht[i][j + 3] *= time_step * time_step;
        }

    // compute left-hand side of inversion part (-Ω_H * Q_a * Ω_H^T)
    float inv[6][6] = {0};
    matrix_mul(3, 3, 3, FLAT(omega_g), FLAT(omega_g), FLAT(temp_mat));
    for (int i = 0; i < 3; i++)
        memcpy(inv[i], temp_mat[i], sizeof(float) * 3);
    
    matrix_mul(3, 3, 3, FLAT(omega_g), FLAT(omega_m), FLAT(temp_mat));
    for (int i = 0; i < 3; i++)
        memcpy(inv[i] + 3, temp_mat[i], sizeof(float) * 3);
    
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            inv[i + 3][j] = inv[j][i + 3];

    matrix_mul(3, 3, 3, FLAT(omega_m), FLAT(omega_m), FLAT(temp_mat));
    for (int i = 0; i < 3; i++)
        memcpy(inv[i + 3] + 3, temp_mat[i], sizeof(float) * 3);

    // subtract (Q_b + R) from above
    float coef_a = uncertainty_linear_accel + noise_accel + time_step * time_step * noise_gyro;
    float coef_m = uncertainty_magnetic_dist + noise_mag + time_step * time_step * noise_gyro;
    for (int i = 0; i < 3; i++)
    {
        inv[i][i] -= coef_a;
        inv[i + 3][i + 3] -= coef_m;
    }

    // compute kalman gain
    float inv2[6][6];
    matrix_inv(6, FLAT(inv), FLAT(inv2));
    matrix_mul(12, 6, 6, FLAT(pht), FLAT(inv2), FLAT(kalman_gain));
}

static void compute_posteriori_error_state(float* accel_read, float* mag_read)
{
    float measurement[6];
    matrix_sub(3, 1, eskf_nominal.gravity, eskf_nominal.linear_accel, measurement);
    matrix_sub(3, 1, accel_read, measurement, measurement);
    
    matrix_add(3, 1, eskf_nominal.earth_magnetic, eskf_nominal.magnetic_dist, measurement + 3);
    matrix_sub(3, 1, mag_read, measurement + 3, measurement + 3);

    matrix_mul(12, 6, 1, FLAT(kalman_gain), measurement, (float*)&eskf_error);
}

static void inject_error_state()
{
    float temp_vec[3];
    float temp_mat[3][3];

    // orientation 
    for (int i = 0; i < 3; i++)
        temp_vec[i] = eskf_error.delta_ori[i] * 0.5f;
    hamilton_product2(eskf_nominal.orientation, 1, temp_vec, eskf_nominal.orientation);

    // angular rate
    matrix_sub(3, 1, eskf_nominal.angular_rate, eskf_error.delta_bias, eskf_nominal.angular_rate);

    // gravity
    matrix_mul_scalar(3, 1, temp_vec, -1.0f);
    small_angle_rotation_matrix(temp_vec, temp_mat);
    memcpy(temp_vec, eskf_nominal.gravity, sizeof(eskf_nominal.gravity));
    matrix_mul(3, 3, 1, FLAT(temp_mat), temp_vec, eskf_nominal.gravity);

    // earth magnetic
    memcpy(temp_vec, eskf_nominal.earth_magnetic, sizeof(eskf_nominal.earth_magnetic));
    matrix_mul(3, 3, 1, FLAT(temp_mat), temp_vec, eskf_nominal.earth_magnetic);

    // angular bias
    matrix_add(3, 1, eskf_nominal.angular_bias, eskf_error.delta_bias, eskf_nominal.angular_bias);

    // linear acceleration
    matrix_add(3, 1, eskf_nominal.linear_accel, eskf_error.delta_accel, eskf_nominal.linear_accel);

    // magnetic disturbance
    matrix_add(3, 1, eskf_nominal.magnetic_dist, eskf_error.delta_dist, eskf_nominal.magnetic_dist);

    // normalize
    normalize_quat(eskf_nominal.orientation);
    normalize_vec(eskf_nominal.gravity);
    normalize_vec(eskf_nominal.earth_magnetic);
}


static void normalize_vec(float* vec)
{
    float coef = 1.0f / sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    vec[0] *= coef;
    vec[1] *= coef;
    vec[2] *= coef;
}

static void normalize_quat(float* quat)
{
    float coef = 1.0f / sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    quat[0] *= coef;
    quat[1] *= coef;
    quat[2] *= coef;
    quat[3] *= coef;
}

static void euler_rodrigues(const float* quat, const float* vec, float* dst)
{
    float skew[3][3];
    float first[3], second[3];
    matrix_skew_symmetrize(&quat[1], FLAT(skew));
    matrix_mul(3, 3, 1, FLAT(skew), vec, first);
    matrix_mul(3, 3, 1, FLAT(skew), first, second);
    for (int i = 0; i < 3; i++)
        dst[i] = vec[i] + 2 * quat[0] * first[i] + 2 * second[i];
}

static void hamilton_product2(const float* lhs, const float w, const float* v, float* dst)
{
    float nw = (lhs[0] * w   ) - (lhs[1] * v[0]) - (lhs[2] * v[1]) - (lhs[3] * v[2]);
    float nx = (lhs[0] * v[0]) + (lhs[1] * w   ) + (lhs[2] * v[2]) - (lhs[3] * v[1]);
    float ny = (lhs[0] * v[1]) - (lhs[1] * v[2]) + (lhs[2] * w   ) + (lhs[3] * v[0]);
    float nz = (lhs[0] * v[2]) + (lhs[1] * v[1]) - (lhs[2] * v[0]) + (lhs[3] * w   );
    dst[0] = nw;
    dst[1] = nx;
    dst[2] = ny;
    dst[3] = nz;
}

static void small_angle_rotation_matrix(const float* vec, float (*dst)[3])
{
    float x = vec[0];
    float y = vec[1];
    float z = vec[2];

    dst[0][0] = 1.0f;
    dst[0][1] = -z;
    dst[0][2] = y;

    dst[1][0] = z;
    dst[1][1] = 1.0f;
    dst[1][2] = -x;

    dst[2][0] = -y;
    dst[2][1] = x;
    dst[2][2] = 1.0f;
}