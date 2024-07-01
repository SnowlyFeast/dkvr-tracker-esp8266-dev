#include "filter/matrix.h"

#include <string.h>

// this macro reconstruct the 2D float array pointer with given 1D-flatten array
// only applicapable on GCC since MSVC doesn't support var-length array
#define RECON_2D_ARR_PTR(type, name, size, ptr)     type(*name)[size] = (type(*)[size])ptr;

void matrix_add(int m, int n, const float* p_lhs, const float* p_rhs, float* p_dst)
{
    RECON_2D_ARR_PTR(const float, lhs, n, p_lhs);
    RECON_2D_ARR_PTR(const float, rhs, n, p_rhs);
    RECON_2D_ARR_PTR(float, dst, n, p_dst);

    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            dst[i][j] = lhs[i][j] + rhs[i][j];
}

void matrix_sub(int m, int n, const float* p_lhs, const float* p_rhs, float* p_dst)
{
    RECON_2D_ARR_PTR(const float, lhs, n, p_lhs);
    RECON_2D_ARR_PTR(const float, rhs, n, p_rhs);
    RECON_2D_ARR_PTR(float, dst, n, p_dst);

    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            dst[i][j] = lhs[i][j] - rhs[i][j];
}

void matrix_mul(int m, int n, int l, const float* p_lhs, const float* p_rhs, float* p_dst)
{
    RECON_2D_ARR_PTR(const float, lhs, n, p_lhs);
    RECON_2D_ARR_PTR(const float, rhs, l, p_rhs);
    RECON_2D_ARR_PTR(float, dst, l, p_dst);

    memset(dst, 0, sizeof(float) * m * l);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < l; j++)
            for (int k = 0; k < n; k++)
                dst[i][j] += lhs[i][k] * rhs[k][j];
}

void matrix_mul_trans(int m, int n, int l, const float* p_lhs, const float* p_rhs, float* p_dst)
{
    RECON_2D_ARR_PTR(const float, lhs, n, p_lhs);
    RECON_2D_ARR_PTR(const float, rhs, n, p_rhs);
    RECON_2D_ARR_PTR(float, dst, l, p_dst);

    memset(dst, 0, sizeof(float) * m * l);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < l; j++)
            for (int k = 0; k < n; k++)
                dst[i][j] += lhs[i][k] * rhs[j][k];
}


void matrix_mul_scalar(int m, int n, float* p_src, float scalar)
{
    RECON_2D_ARR_PTR(float, src, n, p_src);

    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            src[i][j] *= scalar;
}

void matrix_inv(int n, float* p_src, float* p_dst)
{
    RECON_2D_ARR_PTR(float, src, n, p_src);
    RECON_2D_ARR_PTR(float, dst, n, p_dst);

    // LU decomposition
    for (int i = 0; i < n; i++)
    {
        float pivot = src[i][i];
        for (int j = i + 1; j < n; j++)
            src[j][i] /= pivot;

        for (int j = i + 1; j < n; j++)
            for (int k = i + 1; k < n; k++)
                src[j][k] -= src[j][i] * src[i][k];
    }

    // set dst with identity matrix (permutation matrix is identity)
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
            dst[i][j] = 0;
        dst[i][i] = 1;
    }

    // solve LUX = I for X
    for (int i = 0; i < n; i++)
    {
        // forward substitution to solve 'LY = I' for Y
        for (int j = 0; j < n; j++)
            for (int k = 0; k < j; k++)
                dst[j][i] -= src[j][k] * dst[k][i];

        // back substitution to solve 'UX = Y' for X
        for (int j = n - 1; j >= 0; j--)
        {
            for (int k = j + 1; k < n; k++)
                dst[j][i] -= src[j][k] * dst[k][i];
            dst[j][i] /= src[j][j];
        }
    }
}

void matrix_inv_33(const float* p_src, float* p_dst)
{
    RECON_2D_ARR_PTR(const float, src, 3, p_src);
    RECON_2D_ARR_PTR(float, dst, 3, p_dst);

    float det = src[0][0] * src[1][1] * src[2][2] + src[0][1] * src[1][2] * src[2][0] + src[0][2] * src[1][0] * src[2][1] 
              - src[0][0] * src[1][2] * src[2][1] - src[0][1] * src[1][0] * src[2][2] - src[0][2] * src[1][1] * src[2][0];

    if (det == 0) return;

    float coef = 1.0f / det;

    dst[0][0] = coef * (src[1][1] * src[2][2] - src[1][2] * src[2][1]);
    dst[0][1] = coef * (src[0][2] * src[2][1] - src[0][1] * src[2][2]);
    dst[0][2] = coef * (src[0][1] * src[1][2] - src[0][2] * src[1][1]);

    dst[1][0] = coef * (src[1][2] * src[2][0] - src[1][0] * src[2][2]);
    dst[1][1] = coef * (src[0][0] * src[2][2] - src[0][2] * src[2][0]);
    dst[1][2] = coef * (src[0][2] * src[1][0] - src[0][0] * src[1][2]);

    dst[2][0] = coef * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);
    dst[2][1] = coef * (src[0][1] * src[2][0] - src[0][0] * src[2][1]);
    dst[2][2] = coef * (src[0][0] * src[1][1] - src[0][1] * src[1][0]);
}

void matrix_inv_66(const float* p_src)
{
    RECON_2D_ARR_PTR(float, src, 6, p_src);

    // sub-block the matrix
    float mat_a[9], mat_b[9], mat_c[9], mat_d[9];
    for (int i = 0; i < 3; i++)
    {
        memcpy(&mat_a[i * 3], &src[i][0], sizeof(float) * 3);
        memcpy(&mat_b[i * 3], &src[i][3], sizeof(float) * 3);
        memcpy(&mat_c[i * 3], &src[i + 3][0], sizeof(float) * 3);
        memcpy(&mat_d[i * 3], &src[i + 3][3], sizeof(float) * 3);
    }

    // calculate Schur complement
    float inv_d[9];
    matrix_inv_33(mat_d, inv_d);

    float schur_comp[9];
    matrix_mul(3, 3, 3, mat_b, inv_d, schur_comp);
    matrix_mul(3, 3, 3, schur_comp, mat_c, mat_d);  // mat_d is not used anymore, so use it as a temp matrix
    matrix_sub(3, 3, mat_a, mat_d, mat_d);
    matrix_inv_33(mat_d, schur_comp);

    // construct the inverse block-matrix
    // mat_a is same as schur_comp
    float min_inv_d_c[9];   // common value between C and D
    matrix_mul(3, 3, 3, inv_d, mat_c, min_inv_d_c);
    matrix_mul_scalar(3, 3, min_inv_d_c, -1.0f);

    // mat_b
    matrix_mul(3, 3, 3, schur_comp, mat_b, mat_d);  // mat_d is TEMP matrix
    matrix_mul(3, 3, 3, mat_d, inv_d, mat_b);
    matrix_mul_scalar(3, 3, mat_b, -1.0f);

    // mat_c
    matrix_mul(3, 3, 3, min_inv_d_c, schur_comp, mat_c);

    // mat_d
    matrix_mul(3, 3, 3, min_inv_d_c, mat_b, mat_d);
    matrix_add(3, 3, inv_d, mat_d, mat_d);

    // write back to src array
    for (int i = 0; i < 3; i++)
    {
        memcpy(&src[i][0], &schur_comp[i * 3], sizeof(float) * 3);  // mat_a result = schur_comp
        memcpy(&src[i][3], &mat_b[i * 3], sizeof(float) * 3);
        memcpy(&src[i + 3][0], &mat_c[i * 3], sizeof(float) * 3);
        memcpy(&src[i + 3][3], &mat_d[i * 3], sizeof(float) * 3);
    }
}

void matrix_skew_symmetrize(const float* src, float* p_dst)
{
    RECON_2D_ARR_PTR(float, dst, 3, p_dst);
    float x = src[0];
    float y = src[1];
    float z = src[2];
    dst[0][0] = 0;
    dst[0][1] = -z;
    dst[0][2] = y;
    dst[1][0] = z;
    dst[1][1] = 0;
    dst[1][2] = -x;
    dst[2][0] = -y;
    dst[2][1] = x;
    dst[2][2] = 0;
}

void matrix_vector_cross(const float* lhs, const float* rhs, float* dst)
{
    dst[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
    dst[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
    dst[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
}