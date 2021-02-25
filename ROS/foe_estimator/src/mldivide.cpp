//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mldivide.cpp
//
//  Code generation for function 'mldivide'
//


// Include files
#include "mldivide.h"
#include "estimateFoECPP.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
void mldivide(const double A[4], const double B[2], double Y[2])
{
  int r1;
  int r2;
  double a21;
  double Y_tmp;
  if (std::abs(A[1]) > std::abs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = A[r2] / A[r1];
  Y_tmp = A[r1 + 2];
  Y[1] = (B[r2] - B[r1] * a21) / (A[r2 + 2] - a21 * Y_tmp);
  Y[0] = (B[r1] - Y[1] * Y_tmp) / A[r1];
}

// End of code generation (mldivide.cpp)
