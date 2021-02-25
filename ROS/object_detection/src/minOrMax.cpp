//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  minOrMax.cpp
//
//  Code generation for function 'minOrMax'
//


// Include files
#include "minOrMax.h"
#include "postEstimateClustersCPP.h"
#include "rt_nonfinite.h"

// Function Definitions
double minimum(const coder::array<double, 1U> &x)
{
  double ex;
  int n;
  n = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      ex = x[0];
    } else if ((x[0] > x[1]) || (rtIsNaN(x[0]) && (!rtIsNaN(x[1])))) {
      ex = x[1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!rtIsNaN(x[0])) {
      idx = 1;
    } else {
      boolean_T exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= x.size(0))) {
        if (!rtIsNaN(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = x[0];
    } else {
      ex = x[idx - 1];
      idx++;
      for (k = idx; k <= n; k++) {
        double d;
        d = x[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
  }

  return ex;
}

// End of code generation (minOrMax.cpp)
