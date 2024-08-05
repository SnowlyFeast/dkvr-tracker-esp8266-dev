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