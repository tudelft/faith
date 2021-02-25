//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  sign.cpp
//
//  Code generation for function 'sign'
//


// Include files
#include "sign.h"
#include "estimateFoECPP.h"
#include "rt_nonfinite.h"

// Function Definitions
void b_sign(coder::array<double, 1U> &x)
{
  int nx;
  nx = x.size(0);
  for (int k = 0; k < nx; k++) {
    double b_x;
    b_x = x[k];
    if (x[k] < 0.0) {
      b_x = -1.0;
    } else if (x[k] > 0.0) {
      b_x = 1.0;
    } else {
      if (x[k] == 0.0) {
        b_x = 0.0;
      }
    }

    x[k] = b_x;
  }
}

// End of code generation (sign.cpp)
