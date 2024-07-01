#include "filter/ekf.h"

#include <math.h>
#include <string.h>

#include "filter/matrix.h"

#define DEG_TO_RAD          0.017453293f
#define EKF_STATE_PREDICT   0
#define EKF_STATE_CORRECT   1

// state representing variables
static int ekf_state;

// FIXME: make it static after debug
float prio_state[4];
float post_state[4];
static float prio_err_covar[4][4];
static float post_err_covar[4][4];

// calculation variables
static float jacob_a[4][4]; // jacobian A is common between state projection and error covariance projection
static float jacob_h[6][4];
static float kalman_gain[4][6];

// noise spectral density
static float gyro_nsd = 0.05f;
static float accel_nsd = 0.03f;
static float mag_nsd = 0.5f;

static float grv_ref[3] = { 0.0f, 0.0f, -1.0f };
static float mag_ref[3];

// protos
static void calculate_mag_ref(const float* accel_cpy, const float* mag_cpy);

static void update_jacob_a(const float *gyro, float dt);
static void update_priori_state();
static void update_priori_error_covar(float dt);

static void update_kalman_gain();
static void update_posteriori_state(const float *accel, const float *mag);
static void update_posteriori_error_covar();

static void normalize_vec(float* vec);
static void normalize_quat(float* quat);
static void euler_rodrigues(const float* quat, const float* vec, float* dst);

void ekf_set_nsd(float g_nsd, float a_nsd, float m_nsd)
{
    gyro_nsd = g_nsd;
    accel_nsd = a_nsd;
    mag_nsd = m_nsd;
}

void ekf_init(const vector3_t* accel, const vector3_t* mag)
{
    // normalize
    float accel_cpy[3], mag_cpy[3];
    memcpy(accel_cpy, accel, sizeof(accel_cpy));
    memcpy(mag_cpy, mag, sizeof(mag_cpy));
    normalize_vec(accel_cpy);
    normalize_vec(mag_cpy);

    calculate_mag_ref(accel_cpy, mag_cpy);

    // find rotation matrix (TRIAD method)
    float mat1[9], mat2[9];
    float temp1[3], temp2[3];
    matrix_vector_cross(grv_ref, mag_ref, temp1);
    matrix_vector_cross(grv_ref, temp1, temp2);
    normalize_vec(temp1);
    normalize_vec(temp2);
    memcpy(mat1 + 0, grv_ref, sizeof(float) * 3);
    memcpy(mat1 + 3, temp1, sizeof(float) * 3);
    memcpy(mat1 + 6, temp2, sizeof(float) * 3);

    matrix_vector_cross(accel_cpy, mag_cpy, temp1);
    matrix_vector_cross(accel_cpy, temp1, temp2);
    normalize_vec(temp1);
    normalize_vec(temp2);
    memcpy(mat2 + 0, accel_cpy, sizeof(float) * 3);
    memcpy(mat2 + 3, temp1, sizeof(float) * 3);
    memcpy(mat2 + 6, temp2, sizeof(float) * 3);

    float rot_mat[3][3];
    matrix_mul_trans(3, 3, 3, mat2, mat1, FLAT(rot_mat));

    // set initial state
    post_state[0] = 0.5f * sqrt(rot_mat[0][0] + rot_mat[1][1] + rot_mat[2][2] + 1);
    float fqw = 4.0f * post_state[0];
    post_state[1] = (rot_mat[2][1] - rot_mat[1][2]) / fqw;
    post_state[2] = (rot_mat[0][2] - rot_mat[2][0]) / fqw;
    post_state[3] = (rot_mat[1][0] - rot_mat[0][1]) / fqw;

    memset(post_err_covar, 0, sizeof(post_err_covar));
    post_err_covar[0][0] = 1.0f;
    post_err_covar[1][1] = 1.0f;
    post_err_covar[2][2] = 1.0f;
    post_err_covar[3][3] = 1.0f;

    ekf_state = EKF_STATE_PREDICT;
}

void ekf_predict(const vector3_t *gyro, float dt)
{
    if (ekf_state != EKF_STATE_PREDICT) return;

    float gyro_cpy[3] = {gyro->x * DEG_TO_RAD, gyro->y * DEG_TO_RAD, gyro->z * DEG_TO_RAD};
    update_jacob_a(gyro_cpy, dt);
    update_priori_state();
    update_priori_error_covar(dt);

    ekf_state = EKF_STATE_CORRECT;
}

void ekf_correct(const vector3_t *accel, const vector3_t *mag, quaternion_t *out)
{
    if (ekf_state != EKF_STATE_CORRECT) return;

    float accel_cpy[3], mag_cpy[3];
    memcpy(accel_cpy, accel, sizeof(accel_cpy));
    memcpy(mag_cpy, mag, sizeof(mag_cpy));
    normalize_vec(accel_cpy);
    normalize_vec(mag_cpy);
    
    update_kalman_gain();
    update_posteriori_state(accel_cpy, mag_cpy);
    update_posteriori_error_covar();
    memcpy(out, post_state, sizeof(quaternion_t));

    ekf_state = EKF_STATE_PREDICT;
}

static void calculate_mag_ref(const float* accel_cpy, const float* mag_cpy)
{
    // rotate
    float cos = sqrtf(0.5f - 0.5f * accel_cpy[2]);
    float sin = sqrtf(0.5f + 0.5f * accel_cpy[2]);
    float size = sqrtf(accel_cpy[0] * accel_cpy[0] + accel_cpy[1] * accel_cpy[1]);
    float quat[4] = { cos, -accel_cpy[1] * sin / size, accel_cpy[0] * sin / size, 0 };
    euler_rodrigues(quat, mag_cpy, mag_ref);

    // remove y component (remove magnetic declination)
    mag_ref[0] = sqrtf(mag_ref[0] * mag_ref[0] + mag_ref[1] * mag_ref[1]);
    mag_ref[1] = 0;
}

static void update_jacob_a(const float *gyro, float dt)
{
    float coef = dt / 2;
    jacob_a[0][0] = 1;
    jacob_a[0][1] = -gyro[0] * coef;
    jacob_a[0][2] = -gyro[1] * coef;
    jacob_a[0][3] = -gyro[2] * coef;

    jacob_a[1][0] = -jacob_a[0][1];
    jacob_a[1][1] = 1;
    jacob_a[1][2] = -jacob_a[0][3];
    jacob_a[1][3] = jacob_a[0][2];

    jacob_a[2][0] = -jacob_a[0][2];
    jacob_a[2][1] = -jacob_a[1][2];
    jacob_a[2][2] = 1;
    jacob_a[2][3] = -jacob_a[0][1];

    jacob_a[3][0] = -jacob_a[0][3];
    jacob_a[3][1] = -jacob_a[1][3];
    jacob_a[3][2] = -jacob_a[2][3];
    jacob_a[3][3] = 1;
}

static void update_priori_state()
{
    // project the state ahead
    matrix_mul(4, 4, 1, FLAT(jacob_a), post_state, prio_state);
    normalize_quat(prio_state);
}

static void update_priori_error_covar(float dt)
{
    // update process noise covar
    float process_noise_covar[4][4];
    float coef = dt * dt * gyro_nsd / 4;
    process_noise_covar[0][0] = (post_state[1] * post_state[1] + post_state[2] * post_state[2] + post_state[3] * post_state[3]) * coef;
    process_noise_covar[0][1] = -(post_state[0] * post_state[1]) * coef;
    process_noise_covar[0][2] = -(post_state[0] * post_state[2]) * coef;
    process_noise_covar[0][3] = -(post_state[0] * post_state[3]) * coef;

    process_noise_covar[1][0] = process_noise_covar[0][1];
    process_noise_covar[1][1] = process_noise_covar[0][0];
    process_noise_covar[1][2] = -(post_state[1] * post_state[2]) * coef;
    process_noise_covar[1][3] = -(post_state[1] * post_state[3]) * coef;

    process_noise_covar[2][0] = process_noise_covar[0][2];
    process_noise_covar[2][1] = process_noise_covar[1][2];
    process_noise_covar[2][2] = process_noise_covar[0][0];
    process_noise_covar[2][3] = -(post_state[2] * post_state[3]) * coef;

    process_noise_covar[3][0] = process_noise_covar[0][3];
    process_noise_covar[3][1] = process_noise_covar[1][3];
    process_noise_covar[3][2] = process_noise_covar[2][3];
    process_noise_covar[3][3] = process_noise_covar[0][0];

    // project the error covanriance ahead
    float temp[4][4];
    matrix_mul(4, 4, 4, FLAT(jacob_a), FLAT(post_err_covar), FLAT(temp));
    matrix_mul_trans(4, 4, 4, FLAT(temp), FLAT(jacob_a), FLAT(prio_err_covar));
    matrix_add(4, 4, FLAT(prio_err_covar), FLAT(process_noise_covar), FLAT(prio_err_covar));
}

static void update_kalman_gain()
{
    // gravity part
    jacob_h[0][0] = 2 * prio_state[2];
    jacob_h[0][1] = -2 * prio_state[3];
    jacob_h[0][2] = 2 * prio_state[0];
    jacob_h[0][3] = -2 * prio_state[1];

    jacob_h[1][0] = -2 * prio_state[1];
    jacob_h[1][1] = -2 * prio_state[0];
    jacob_h[1][2] = -2 * prio_state[3];
    jacob_h[1][3] = -2 * prio_state[2];

    jacob_h[2][0] = -2 * prio_state[0];
    jacob_h[2][1] = 2 * prio_state[1];
    jacob_h[2][2] = 2 * prio_state[2];
    jacob_h[2][3] = -2 * prio_state[3];

    // mag part
    jacob_h[3][0] = 2 * (mag_ref[0] * prio_state[0] - mag_ref[2] * prio_state[2]);
    jacob_h[4][0] = 2 * (-mag_ref[0] * prio_state[3] + mag_ref[2] * prio_state[1]);
    jacob_h[5][0] = 2 * (mag_ref[0] * prio_state[2] + mag_ref[2] * prio_state[0]);

    jacob_h[3][1] = 2 * (mag_ref[0] * prio_state[1] + mag_ref[2] * prio_state[3]);
    jacob_h[3][2] = 2 * (-mag_ref[0] * prio_state[2] - mag_ref[2] * prio_state[0]);
    jacob_h[3][3] = 2 * (-mag_ref[0] * prio_state[3] + mag_ref[2] * prio_state[1]);

    jacob_h[4][1] = -jacob_h[3][2];
    jacob_h[4][2] = jacob_h[3][1];
    jacob_h[4][3] = 2 * (-mag_ref[0] * prio_state[0] + mag_ref[2] * prio_state[2]);

    jacob_h[5][1] = -jacob_h[3][3];
    jacob_h[5][2] = -jacob_h[4][3];
    jacob_h[5][3] = jacob_h[3][1];

    // process kalman gain
    float lhs_mat[4][6] = {}; // reuse lhs_mat after inverse
    matrix_mul_trans(4, 4, 6, FLAT(prio_err_covar), FLAT(jacob_h), FLAT(lhs_mat));

    float temp[6][6];
    matrix_mul(6, 4, 6, FLAT(jacob_h), FLAT(lhs_mat), FLAT(temp));
    temp[0][0] += accel_nsd;
    temp[1][1] += accel_nsd;
    temp[2][2] += accel_nsd;
    temp[3][3] += mag_nsd;
    temp[4][4] += mag_nsd;
    temp[5][5] += mag_nsd;
    matrix_inv_66(FLAT(temp));

    matrix_mul(4, 6, 6, FLAT(lhs_mat), FLAT(temp), FLAT(kalman_gain));
}

static void update_posteriori_state(const float *accel, const float *mag)
{
    float quat_inv[4] = {prio_state[0], -prio_state[1], -prio_state[2], -prio_state[3]};
    float predicted_grv[3], predicted_mag[3];
    euler_rodrigues(quat_inv, grv_ref, predicted_grv);
    euler_rodrigues(quat_inv, mag_ref, predicted_mag);

    float residual[6];
    for (int i = 0; i < 3; i++)
    {
        residual[i + 0] = accel[i] - predicted_grv[i];
        residual[i + 3] = mag[i] - predicted_mag[i];
    }

    matrix_mul(4, 6, 1, FLAT(kalman_gain), residual, post_state);
    matrix_add(4, 1, post_state, prio_state, post_state);

    normalize_quat(post_state);
}

static void update_posteriori_error_covar()
{
    float post_covar_lhs[4][4];
    matrix_mul(4, 6, 4, FLAT(kalman_gain), FLAT(jacob_h), FLAT(post_covar_lhs));
    post_covar_lhs[0][0] -= 1.0f;
    post_covar_lhs[1][1] -= 1.0f;
    post_covar_lhs[2][2] -= 1.0f;
    post_covar_lhs[3][3] -= 1.0f;
    matrix_mul_scalar(4, 4, FLAT(post_covar_lhs), -1.0f);
    matrix_mul(4, 4, 4, FLAT(post_covar_lhs), FLAT(prio_err_covar), FLAT(post_err_covar));
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