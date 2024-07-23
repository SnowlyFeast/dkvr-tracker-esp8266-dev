#include "filter/eskf_sensor_fusion.h"

#include <math.h>
#include <string.h>

#include "filter/matrix.h"

// state variable
int eskf_ready = 0;
struct eskf_nominal_state eskf_nominal;
struct eskf_error_state eskf_error;

// configuration variable
static float time_step = 0.01f;
static float noise_gyro[3] = {2.74e-7f, 2.74e-7f, 2.74e-7f};
static float noise_accel[3] = {4.0e-6f, 4.0e-6f, 4.0e-6f};
static float noise_mag[3] = {4.0e-6f, 4.0e-6f, 4.0e-6f};
static float uncertainty_linear_accel = 0.04f;
static float uncertainty_magnetic_dist = 0.08f;
static float lpfc_linear_accel = 0.5f;
static float lpfc_magnetic_dist = 0.5f;

static const float linear_accel_threshold = 0.2f * 0.2f;
static const float magnetic_dist_threshold = 0.3f * 0.3f;


// computation variable
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

// protos
static void update_priori_state(float* gyro_read);
static void predict_linear_accel_and_magnetic_dist(float* accel_read, float* mag_read);
static void compute_kalman_gain();
static void update_posteriori_error_covar();
static void update_and_inject_error_state(float* accel_read, float* mag_read);

static void normalize_vec(float* vec);
static void normalize_quat(float* quat);
static void euler_rodrigues(const float* quat, const float* vec, float* dst);
static void hamilton_product2(const float* lhs, const float w, const float* v, float* dst);

// FIXME: remove after debug
#include "common/system_interface.h"
static void serial_print_vector(float* v, int n)
{
    static int count = 0;
    if (count++ < 3) return; else count = 0;

    for (int i = 0; i < n; i++)
        dkvr_serial_print_float(v[i]);
    dkvr_serial_print("\r\n");
}

void eskf_configure(struct eskf_configuration *config)
{
    time_step = config->time_step;
    memcpy(noise_gyro, config->noise_gyro, sizeof(noise_gyro));
    memcpy(noise_accel, config->noise_accel,sizeof(noise_accel));
    memcpy(noise_mag, config->noise_mag, sizeof(noise_mag));
    uncertainty_linear_accel = config->uncertainty_linear_accel;
    uncertainty_magnetic_dist = config->uncertainty_magnetic_dist;
    lpfc_linear_accel = expf(-2 * M_PI * time_step * config->lpf_cutoff_linear_accel);
    lpfc_magnetic_dist = expf(-2 * M_PI * time_step * config->lpf_cutoff_magnetic_dist);

    // update observation error covariance matrix
    for (int i = 0; i < 3; i++)
    {
        observation_error[0][i] = noise_accel[i] + uncertainty_linear_accel + time_step * time_step * noise_gyro[i];
        observation_error[1][i] = noise_mag[i] + uncertainty_magnetic_dist + time_step * time_step * noise_gyro[i];
    }
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
    
    // set orientation
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

void eskf_update(struct eskf_vector3 gyro_read, struct eskf_vector3 accel_read, struct eskf_vector3 mag_read)
{
    if (!eskf_ready) return;
    update_priori_state(gyro_read.data);
    predict_linear_accel_and_magnetic_dist(accel_read.data, mag_read.data);
    compute_kalman_gain();
    update_posteriori_error_covar();
    update_and_inject_error_state(accel_read.data, mag_read.data);
}

static void update_priori_state(float* gyro_read)
{
    float angular_rate[3];
    for (int i = 0; i < 3; i++)
        angular_rate[i] = gyro_read[i] * time_step * 0.5f;
    
    hamilton_product2(eskf_nominal.orientation, 1, angular_rate, eskf_nominal.orientation);
    matrix_mul_scalar(3, 1, eskf_nominal.linear_accel, lpfc_linear_accel);
    matrix_mul_scalar(3, 1, eskf_nominal.magnetic_dist, lpfc_magnetic_dist);

    // update priori error covariance
    float rot_mat[3][3] = {
        {                1,  angular_rate[2], -angular_rate[1] },
        { -angular_rate[2],                1,  angular_rate[0] },
        {  angular_rate[1], -angular_rate[0],                1 }
    };
    float temp[9][3];
    memcpy(temp, error_covar, sizeof(float) * 3 * 9);
    matrix_mul(3, 3, 9, FLAT(rot_mat), FLAT(temp), FLAT(error_covar));

    for (int i = 0; i < 9; i++)
        memcpy(temp[i], error_covar[i], sizeof(float) * 3);
    float result[9][3];
    matrix_mul_trans(9, 3, 3, FLAT(temp), FLAT(error_covar), FLAT(result));
    for (int i = 0; i < 9; i++)
        memcpy(error_covar[i], temp[i], sizeof(float) * 3);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 9; j++)
        {
            error_covar[i + 3][j] *= lpfc_linear_accel;
            error_covar[j][i + 3] *= lpfc_linear_accel;
            error_covar[i + 6][j] *= lpfc_magnetic_dist;
            error_covar[j][i + 6] *= lpfc_magnetic_dist;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        error_covar[i][i] += time_step * time_step * noise_gyro[i];
        error_covar[i + 3][i + 3] += uncertainty_linear_accel;
        error_covar[i + 6][i + 6] += uncertainty_magnetic_dist;
    }
}

static void predict_linear_accel_and_magnetic_dist(float* accel_read, float* mag_read)
{
    float linear_accel[3], magnetic_dist[3];
    euler_rodrigues(eskf_nominal.orientation, accel_read, linear_accel);
    matrix_sub(3, 1, linear_accel, gravity_ref, linear_accel);

    euler_rodrigues(eskf_nominal.orientation, mag_read, magnetic_dist);
    matrix_sub(3, 1, magnetic_dist, magnetic_ref, magnetic_dist);

    float linear_accel_magnitude = powf(linear_accel[0], 2) + powf(linear_accel[1], 2) + powf(linear_accel[2], 2);
    float magnetic_dist_magnitude = powf(magnetic_dist[0], 2) + powf(magnetic_dist[1], 2) + powf(magnetic_dist[2], 2);

    ignore_linear_accel = (linear_accel_magnitude < linear_accel_threshold);
    ignore_magnetic_dist = (magnetic_dist_magnitude < magnetic_dist_threshold);

    if (ignore_linear_accel)  normalize_vec(accel_read);
    if (ignore_magnetic_dist) normalize_vec(mag_read);
}

static void compute_kalman_gain()
{
    // update expected acceleration and magnetic
    float inv_quat[4] = {eskf_nominal.orientation[0], -eskf_nominal.orientation[1],
                         -eskf_nominal.orientation[2], -eskf_nominal.orientation[3]};

    matrix_sub(3, 1, gravity_ref, eskf_nominal.linear_accel, estimated_accel);
    euler_rodrigues(inv_quat, estimated_accel, estimated_accel);

    matrix_add(3, 1, magnetic_ref, eskf_nominal.magnetic_dist, estimated_magnetic);
    euler_rodrigues(inv_quat, estimated_magnetic, estimated_magnetic);

    // update observation matrix
    observation_matrix[0][1] = -estimated_accel[2];
    observation_matrix[0][2] =  estimated_accel[1];
    observation_matrix[1][0] =  estimated_accel[2];
    observation_matrix[1][2] = -estimated_accel[0];
    observation_matrix[2][0] = -estimated_accel[1];
    observation_matrix[2][1] =  estimated_accel[0];

    observation_matrix[3][1] = -estimated_magnetic[2];
    observation_matrix[3][2] =  estimated_magnetic[1];
    observation_matrix[4][0] =  estimated_magnetic[2];
    observation_matrix[4][2] = -estimated_magnetic[0];
    observation_matrix[5][0] = -estimated_magnetic[1];
    observation_matrix[5][1] =  estimated_magnetic[0];

    for (int i = 0; i < 3; i++)
    {
        observation_matrix[i][i + 3] = ignore_linear_accel ? 0.0f : -1.0f;
        observation_matrix[i + 3][i + 6] = ignore_magnetic_dist ? 0.0f : 1.0f;
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
        result[i][i] -= 1.0f;

    matrix_mul(9, 9, 9, FLAT(temp), FLAT(error_covar), FLAT(result));
    matrix_mul_trans(9, 9, 9, FLAT(result), FLAT(temp), FLAT(error_covar));    

    float msr_error[6][6] = {{0}};
    for (int i = 0; i < 3; i++)
    {
        msr_error[i][i] = observation_error[0][i];
        msr_error[i + 3][i + 3] = observation_error[1][i];
    }
    matrix_mul(9, 6, 6, FLAT(kalman_gain), FLAT(msr_error), FLAT(temp));
    matrix_mul_trans(6, 6, 9, FLAT(temp), FLAT(kalman_gain), FLAT(result));
    matrix_add(9, 9, FLAT(error_covar), FLAT(result), FLAT(error_covar));
}

static void update_and_inject_error_state(float* accel_read, float* mag_read)
{
    float measurement[6];
    matrix_sub(3, 1, accel_read, estimated_accel, measurement);
    matrix_sub(3, 1, mag_read, estimated_magnetic, measurement + 3);
    matrix_mul(9, 6, 1, FLAT(kalman_gain), measurement, (float*)&eskf_error);

    float temp_vec[3];
    for (int i = 0; i < 3; i++)
        temp_vec[i] = eskf_error.orientation_error[i] * 0.5f;
    
    hamilton_product2(eskf_nominal.orientation, 1, temp_vec, eskf_nominal.orientation);
    normalize_quat(eskf_nominal.orientation);
    matrix_add(3, 1, eskf_nominal.linear_accel, eskf_error.linear_accel_error, eskf_nominal.linear_accel);
    matrix_add(3, 1, eskf_nominal.magnetic_dist, eskf_error.magnetic_dist_error, eskf_nominal.magnetic_dist);

    serial_print_vector(eskf_error.magnetic_dist_error, 3);
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