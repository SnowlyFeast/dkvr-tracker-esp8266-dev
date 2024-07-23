#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// flatten given 2d array to 1d array
#define FLAT(x)     (x)[0]

// perform (m x n) matrix element-wise addition.
// it's safe to use the operands as a dst.
void matrix_add(int m, int n, const float* lhs, const float* rhs, float* dst);

// perform (m x n) matrix element-wise subtraction.
// it's safe to use the operands as a dst.
void matrix_sub(int m, int n, const float* lhs, const float* rhs, float* dst);

// perform (m x n) * (n x l) matrix multiplication.
// it's UNSAFE to use the operands as a dst.
void matrix_mul(int m, int n, int l, const float* lhs, const float* rhs, float* dst);

// perform (m x n) * (l * n)^T matrix multiplication.
// it's UNSAFE to use the operands as a dst.
void matrix_mul_trans(int m, int n, int l, const float* lhs, const float* rhs, float* dst);

// multiply the scalar to every member of src.
// result will overwrite the src matrix.
void matrix_mul_scalar(int m, int n, float* src, float scalar);

// perform (n x n) matrix inversion with LU decomposition.
// src matrix is used as temp matrix, overwrited with LU decomposed value.
// this function does not perform pivoting, thus diagonal element must not be zero.
void matrix_inv(int n, float* src, float* dst);

// perform (3 x 3) matrix inversion.
// it's UNSAFE to use the operands as ad dst
void matrix_inv_33(const float* src, float* dst);

// perform (6 x 6) matrix inversion.
// result will overwrite the src matrix
void matrix_inv_66(const float* src);

// create skew symmetric metrix with given vector.
// only 3 front elements of src array is used.
// it's SAFE to use the src as a dst.
void matrix_skew_symmetrize(const float* src, float* dst);

// perfrom a cross product from two given (3x1) vectors.
void matrix_vector_cross(const float* lhs, const float* rhs, float* dst);

#ifdef __cplusplus
}
#endif