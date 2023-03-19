/*--------------------------------------------------------------------------//
Name : Ian Syndergaard
Date: 3/18/2023
Description: Functions for 3x3 matrix inversion and multiplication of a 3x3
    matrix with a 3x1 vector. Functions supplied by interviewer from Intuitive
//--------------------------------------------------------------------------*/

#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

// Direct inverse of a 3x3 matrix m
void mat3x3_inv(double minv[3][3], const double m[3][3])
{
    double det;
    double invdet;
    det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    invdet = 1.0 / det;
    minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
    minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
    minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
    minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
    minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
    minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
    minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
    minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
    minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
}

// Explicit 3x3 matrix * vector. b = A x where A is 3x3
void mat3x3_vec_mult(double b[3], const double A[3][3], const double x[3])
{
    const int N = 3;
    int idx, jdx;
    for( idx = 0; idx < N; idx++ )
    {
        b[idx] = 0.0;
        for( jdx = 0; jdx < N; jdx++)
            {
            b[idx] += A[idx][jdx] * x[jdx] ;
            }
    }
}

#endif