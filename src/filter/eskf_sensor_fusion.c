#include "filter/eskf_sensor_fusion.h"

#include <math.h>
#include <string.h>

#include "filter/matrix.h"

#define SQUARE(x) (x * x)

// state variable
int eskf_ready = 0;
struct eskf_nominal_state eskf_nominal;
struct eskf_error_state eskf_error;

// configuration variable
static float time_step      = 0.01f;
static float noise_gyro[3]  = {2.74e-7f, 2.74e-7f, 2.74e-7f};
static float noise_accel[3] = {4.0e-6f, 4.0e-6f, 4.0e-6f};
static float noise_mag[3]   = {4.0e-6f, 4.0e-6f, 4.0e-6f};
static float uncertainty_linear_accel    = 0.01f;
static float uncertainty_magnetic_dist   = 0.01f;
static float uncertainty_orientation_low = 1.745329e-4f;
static float uncertainty_orientation_med = 3.490659e-3f;
static float lpfc_linear_accel  = 0.5f;
static float lpfc_magnetic_dist = 0.5f;

static const float linear_accel_threshold = SQUARE(0.2f);
static const float magnetic_dist_threshold = SQUARE(0.2f);

// computation variable
static float raw_gyr[3], raw_acc[3], raw_mag[3];
static float gravity_ref[3] = { 0.0f, 0.0f, -1.0f };
static float magnetic_ref[3];
static float estimated_accel[3];
static float estimated_magnetic[3];
static float observation_matrix[6][9];
static float observation_error[2][3];
static float error_covar[9][9];
static float kalman_gain[9][6];
static int ignore_linear_accel;
static int ignore_magnetic_dist;
static float rotation_matrix[3][3];

// protos
static void update_priori_state();
static void predict_linear_accel_and_magnetic_dist();
static void compute_kalman_gain();
static void update_posteriori_error_covar();
static void update_and_inject_error_state();

static void normalize_vec(float* vec);
static void normalize_quat(float* quat);
static void euler_rodrigues(const float* quat, const float* vec, float* dst);
static void hamilton_product2(const float* lhs, const float w, const float* v, float* dst);
static void quat_to_rotation_matrix(const float* quat);

void eskf_configure(const struct eskf_configuration *config)
{
    time_step = config->time_step;
    memcpy(noise_gyro, config->noise_gyro, sizeof(noise_gyro));
    memcpy(noise_accel, config->noise_accel,sizeof(noise_accel));
    memcpy(noise_mag, config->noise_mag, sizeof(noise_mag));
    uncertainty_linear_accel = config->uncertainty_linear_accel;
    uncertainty_magnetic_dist = config->uncertainty_magnetic_dist;
    uncertainty_orientation_low = config->uncertainty_orientaiton_low;
    uncertainty_orientation_med = config->uncertainty_orientation_med;
    lpfc_linear_accel  = expf(-2 * M_PI * time_step * config->lpf_cutoff_linear_accel);
    lpfc_magnetic_dist = expf(-2 * M_PI * time_step * config->lpf_cutoff_magnetic_dist);

    // update observation error covariance matrix
    for (int i = 0; i < 3; i++)
    {
        observation_error[0][i] = noise_accel[i];
        observation_error[1][i] = noise_mag[i];
    }
}

void eskf_init(const float* acc_read, const float* mag_read)
{
    // normalize
    memcpy(raw_acc, acc_read, sizeof(raw_acc));
    memcpy(raw_mag, mag_read, sizeof(raw_mag));
    normalize_vec(raw_acc);
    normalize_vec(raw_mag);

    // calculate mag ref
    float cos = sqrtf(0.5f - 0.5f * raw_acc[2]);
    float sin = sqrtf(0.5f + 0.5f * raw_acc[2]);
    float size = sqrtf(raw_acc[0] * raw_acc[0] + raw_acc[1] * raw_acc[1]);
    float quat[4] = { cos, -raw_acc[1] * sin / size, raw_acc[0] * sin / size, 0 };
    euler_rodrigues(quat, raw_mag, magnetic_ref);

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

    matrix_vector_cross(raw_acc, raw_mag, temp1);
    matrix_vector_cross(raw_acc, temp1, temp2);
    normalize_vec(temp1);
    normalize_vec(temp2);
    memcpy(mat2 + 0, raw_acc, sizeof(float) * 3);
    memcpy(mat2 + 3, temp1, sizeof(float) * 3);
    memcpy(mat2 + 6, temp2, sizeof(float) * 3);

    float rot_mat[3][3];
    matrix_mul_trans(3, 3, 3, mat2, mat1, FLAT(rot_mat));

    // calculate initial orientation
    quat[0] = 0.5f * sqrtf(rot_mat[0][0] + rot_mat[1][1] + rot_mat[2][2] + 1);
    float fqw = 4.0f * quat[0];
    quat[1] = (rot_mat[2][1] - rot_mat[1][2]) / fqw;
    quat[2] = (rot_mat[0][2] - rot_mat[2][0]) / fqw;
    quat[3] = (rot_mat[1][0] - rot_mat[0][1]) / fqw;
    
    // set initial orientation
    memcpy(eskf_nominal.orientation, quat, sizeof(quat));

    // reset angular rate, bias, linear acceleration and magnetic disturbance
    for (int i = 0; i < 3; i++)
    {
        eskf_nominal.linear_accel[i] = 0.0f;
        eskf_nominal.magnetic_dist[i] = 0.0f;
    }

    // reset error covar
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++)
            error_covar[i][j] = 0.0f;
        error_covar[i][i] = 1.0f;
    }
    
    eskf_ready = 1;
}

void eskf_update(const float* gyr_read, const float* acc_read, const float* mag_read)
{
    if (!eskf_ready) return;

    memcpy(raw_gyr, gyr_read, sizeof(raw_gyr));
    memcpy(raw_acc, acc_read, sizeof(raw_acc));
    memcpy(raw_mag, mag_read, sizeof(raw_mag));

    update_priori_state();
    predict_linear_accel_and_magnetic_dist();
    compute_kalman_gain();
    update_posteriori_error_covar();
    update_and_inject_error_state();
}

static void update_priori_state()
{
    // update priori error state
    float magnitude = sqrtf(raw_gyr[0] * raw_gyr[0] + raw_gyr[1] * raw_gyr[1] + raw_gyr[2] * raw_gyr[2]);
    float cos = cosf(magnitude * time_step * 0.5f);
    float sin = sinf(magnitude * time_step * 0.5f) / magnitude;
    float delta_quat[4] = {cos, raw_gyr[0] * sin, raw_gyr[1] * sin, raw_gyr[2] * sin};

    hamilton_product2(eskf_nominal.orientation, delta_quat[0], delta_quat + 1, eskf_nominal.orientation);
    matrix_mul_scalar(3, 1, eskf_nominal.linear_accel, lpfc_linear_accel);
    matrix_mul_scalar(3, 1, eskf_nominal.magnetic_dist, lpfc_magnetic_dist);

    // update priori error covariance
    quat_to_rotation_matrix(delta_quat);

    float temp[9][3];
    memcpy(temp, error_covar, sizeof(float) * 3 * 9);                           // extract top 3 row
    matrix_mul(3, 3, 9, FLAT(rotation_matrix), FLAT(temp), FLAT(error_covar));  // multiplied to (rotation matrix)^T

    float result[9][3];
    for (int i = 0; i < 9; i++)
        memcpy(temp[i], error_covar[i], sizeof(float) * 3);                     // extract left 3 column
    matrix_mul_trans(9, 3, 3, FLAT(temp), FLAT(rotation_matrix), FLAT(result)); // multiplied by (rotation matrix)
    for (int i = 0; i < 9; i++)
        memcpy(error_covar[i], result[i], sizeof(float) * 3);                   // write back

    for (int i = 0; i < 3; i++) {                                               // multiply damping factor
        for (int j = 0; j < 9; j++)
        {
            error_covar[i + 3][j    ] *= lpfc_linear_accel;
            error_covar[j    ][i + 3] *= lpfc_linear_accel;
            error_covar[i + 6][j    ] *= lpfc_magnetic_dist;
            error_covar[j    ][i + 6] *= lpfc_magnetic_dist;
        }
    }

    // add noise & uncertainty variance
    for (int i = 0; i < 3; i++)
    {
        error_covar[i][i]         += time_step * time_step * noise_gyro[i];
        error_covar[i + 3][i + 3] += time_step * uncertainty_linear_accel;
        error_covar[i + 6][i + 6] += time_step * uncertainty_magnetic_dist;
    }
}

static void predict_linear_accel_and_magnetic_dist()
{
    // predict size by magnitude
    float inv_quat[4] = { eskf_nominal.orientation[0], -eskf_nominal.orientation[1],
                         -eskf_nominal.orientation[2], -eskf_nominal.orientation[3]};
    euler_rodrigues(inv_quat, gravity_ref, estimated_accel);
    euler_rodrigues(inv_quat, magnetic_ref, estimated_magnetic);

    matrix_sub(3, 1, raw_acc, estimated_accel, estimated_accel);
    matrix_sub(3, 1, raw_mag, estimated_magnetic, estimated_magnetic);

    float p_linear_accel  = powf(estimated_accel[0], 2)    + powf(estimated_accel[1], 2)    + powf(estimated_accel[2], 2);
    float p_magnetic_dist = powf(estimated_magnetic[0], 2) + powf(estimated_magnetic[1], 2) + powf(estimated_magnetic[2], 2);

    ignore_linear_accel = (p_linear_accel < linear_accel_threshold);
    ignore_magnetic_dist = (p_magnetic_dist < magnetic_dist_threshold);

    if (ignore_linear_accel)  normalize_vec(raw_acc);
    if (ignore_magnetic_dist) normalize_vec(raw_mag);

    // inject orientation uncertainty
    float uncert = (!ignore_linear_accel || !ignore_magnetic_dist) ? uncertainty_orientation_low : uncertainty_orientation_med;
    uncert *= time_step;

    for (int i = 0; i < 3; i++)
        error_covar[i][i] += uncert * uncert;
}

static void compute_kalman_gain()
{
    // update estimated acceleration and magnetic
    float inv_quat[4] = { eskf_nominal.orientation[0], -eskf_nominal.orientation[1],
                         -eskf_nominal.orientation[2], -eskf_nominal.orientation[3]};

    if (ignore_linear_accel)
        memcpy(estimated_accel, gravity_ref, sizeof(gravity_ref));
    else
        matrix_sub(3, 1, gravity_ref, eskf_nominal.linear_accel, estimated_accel);
    euler_rodrigues(inv_quat, estimated_accel, estimated_accel);

    if (ignore_magnetic_dist)
        memcpy(estimated_magnetic, magnetic_ref, sizeof(magnetic_ref));
    else
        matrix_add(3, 1, magnetic_ref, eskf_nominal.magnetic_dist, estimated_magnetic);
    euler_rodrigues(inv_quat, estimated_magnetic, estimated_magnetic);

    // update observation matrix
    float temp_vec[3];
    float temp_mat[3][3];

    memcpy(temp_vec, estimated_accel, sizeof(float) * 3);
    matrix_mul_scalar(3, 1, temp_vec, 0.5f);
    matrix_skew_symmetrize(temp_vec, FLAT(temp_mat));
    for (int i = 0; i < 3; i++)
        memcpy(observation_matrix[i], temp_mat[i], sizeof(float) * 3);
    
    memcpy(temp_vec, estimated_magnetic, sizeof(float) * 3);
    matrix_mul_scalar(3, 1, temp_vec, 0.5f);
    matrix_skew_symmetrize(temp_vec, FLAT(temp_mat));
    for (int i = 0; i < 3; i++)
        memcpy(observation_matrix[i + 3], temp_mat[i], sizeof(float) * 3);

    quat_to_rotation_matrix(inv_quat);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            observation_matrix[i    ][j + 3] = ignore_linear_accel  ? 0 : -rotation_matrix[i][j];
            observation_matrix[i + 3][j + 6] = ignore_magnetic_dist ? 0 :  rotation_matrix[i][j];
        }

    // compute kalman gain
    float pht[9][6];
    matrix_mul_trans(9, 9, 6, FLAT(error_covar), FLAT(observation_matrix), FLAT(pht));

    float inv[6][6], result[9][9];
    matrix_mul(6, 9, 6, FLAT(observation_matrix), FLAT(pht), FLAT(inv));
    for (int i = 0; i < 3; i++)
    {
        inv[i][i]         += observation_error[0][i];
        inv[i + 3][i + 3] += observation_error[1][i];
    }
    matrix_inv(6, FLAT(inv), FLAT(result));
    matrix_mul(9, 6, 6, FLAT(pht), FLAT(result), FLAT(kalman_gain));
}

static void update_posteriori_error_covar()
{
    float temp[9][9], result[9][9];
    matrix_mul(9, 6, 9, FLAT(kalman_gain), FLAT(observation_matrix), FLAT(temp));
    for (int i = 0; i < 9; i++)
        temp[i][i] -= 1.0f;

    matrix_mul(9, 9, 9, FLAT(temp), FLAT(error_covar), FLAT(result));
    matrix_mul_trans(9, 9, 9, FLAT(result), FLAT(temp), FLAT(error_covar));    

    float msr_error[6][6] = {{0}};
    for (int i = 0; i < 3; i++)
    {
        msr_error[i][i] = observation_error[0][i];
        msr_error[i + 3][i + 3] = observation_error[1][i];
    }
    matrix_mul(9, 6, 6, FLAT(kalman_gain), FLAT(msr_error), FLAT(temp));
    matrix_mul_trans(9, 6, 9, FLAT(temp), FLAT(kalman_gain), FLAT(result));
    matrix_add(9, 9, FLAT(error_covar), FLAT(result), FLAT(error_covar));
}

static void update_and_inject_error_state()
{
    float measurement[6];
    matrix_sub(3, 1, raw_acc, estimated_accel, measurement);
    matrix_sub(3, 1, raw_mag, estimated_magnetic, measurement + 3);
    matrix_mul(9, 6, 1, FLAT(kalman_gain), measurement, (float*)&eskf_error);

    float temp_vec[3];
    for (int i = 0; i < 3; i++)
        temp_vec[i] = eskf_error.orientation_error[i] * 0.5f;
    
    hamilton_product2(eskf_nominal.orientation, 1, temp_vec, eskf_nominal.orientation);
    normalize_quat(eskf_nominal.orientation);
    matrix_add(3, 1, eskf_nominal.linear_accel, eskf_error.linear_accel_error, eskf_nominal.linear_accel);
    matrix_add(3, 1, eskf_nominal.magnetic_dist, eskf_error.magnetic_dist_error, eskf_nominal.magnetic_dist);
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

static void quat_to_rotation_matrix(const float* quat)
{
    matrix_skew_symmetrize(&quat[1], FLAT(rotation_matrix));
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rotation_matrix[i][j] *= 2 * quat[0];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            rotation_matrix[i][j] = 2 * quat[i + 1] * quat[j + 1];
    
    float diagonal = powf(quat[0], 2) - (powf(quat[1], 2) + powf(quat[2], 2) + powf(quat[3], 2));
    for (int i = 0; i < 3; i++)
        rotation_matrix[i][i] += diagonal;
}