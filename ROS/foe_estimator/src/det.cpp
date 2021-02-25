//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  det.cpp
//
//  Code generation for function 'det'
//


// Include files
#include "det.h"
#include "estimateFoECPP.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
double det(const double x[4])
{
  double y;
  double x_idx_0;
  double x_idx_1;
  double x_idx_2;
  double x_idx_3;
  signed char ipiv_idx_0;
  int a;
  x_idx_0 = x[0];
  x_idx_1 = x[1];
  x_idx_2 = x[2];
  x_idx_3 = x[3];
  ipiv_idx_0 = 1;
  a = 0;
  if (std::abs(x[1]) > std::abs(x[0])) {
    a = 1;
  }

  if (x[a] != 0.0) {
    if (a != 0) {
      ipiv_idx_0 = 2;
      x_idx_0 = x[1];
      x_idx_1 = x[0];
      x_idx_2 = x[3];
      x_idx_3 = x[2];
    }

    x_idx_1 /= x_idx_0;
  }

  if (x_idx_2 != 0.0) {
    x_idx_3 += x_idx_1 * -x_idx_2;
  }

  y = x_idx_0 * x_idx_3;
  if (ipiv_idx_0 > 1) {
    y = -y;
  }

  return y;
}

// End of code generation (det.cpp)
