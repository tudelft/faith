//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  median.cpp
//
//  Code generation for function 'median'
//


// Include files
#include "median.h"
#include "prepEstimateClustersCPP.h"
#include "quickselect.h"
#include "rt_nonfinite.h"

// Function Definitions
double median(const coder::array<double, 1U> &x)
{
  double y;
  int vlen;
  int k;
  coder::array<double, 1U> unusedU3;
  int unusedU5;
  double b;
  vlen = x.size(0);
  if (x.size(0) == 0) {
    y = rtNaN;
  } else {
    k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (k <= vlen - 1) {
        if (rtIsNaN(x[k])) {
          y = rtNaN;
          exitg1 = 1;
        } else {
          k++;
        }
      } else {
        if (vlen <= 4) {
          if (vlen == 0) {
            y = rtNaN;
          } else if (vlen == 1) {
            y = x[0];
          } else if (vlen == 2) {
            if (((x[0] < 0.0) != (x[1] < 0.0)) || rtIsInf(x[0])) {
              y = (x[0] + x[1]) / 2.0;
            } else {
              y = x[0] + (x[1] - x[0]) / 2.0;
            }
          } else if (vlen == 3) {
            if (x[0] < x[1]) {
              if (x[1] < x[2]) {
                unusedU5 = 1;
              } else if (x[0] < x[2]) {
                unusedU5 = 2;
              } else {
                unusedU5 = 0;
              }
            } else if (x[0] < x[2]) {
              unusedU5 = 0;
            } else if (x[1] < x[2]) {
              unusedU5 = 2;
            } else {
              unusedU5 = 1;
            }

            y = x[unusedU5];
          } else {
            if (x[0] < x[1]) {
              if (x[1] < x[2]) {
                k = 0;
                unusedU5 = 1;
                vlen = 2;
              } else if (x[0] < x[2]) {
                k = 0;
                unusedU5 = 2;
                vlen = 1;
              } else {
                k = 2;
                unusedU5 = 0;
                vlen = 1;
              }
            } else if (x[0] < x[2]) {
              k = 1;
              unusedU5 = 0;
              vlen = 2;
            } else if (x[1] < x[2]) {
              k = 1;
              unusedU5 = 2;
              vlen = 0;
            } else {
              k = 2;
              unusedU5 = 1;
              vlen = 0;
            }

            if (x[k] < x[3]) {
              if (x[3] < x[vlen]) {
                if (((x[unusedU5] < 0.0) != (x[3] < 0.0)) || rtIsInf(x[unusedU5]))
                {
                  y = (x[unusedU5] + x[3]) / 2.0;
                } else {
                  y = x[unusedU5] + (x[3] - x[unusedU5]) / 2.0;
                }
              } else if (((x[unusedU5] < 0.0) != (x[vlen] < 0.0)) || rtIsInf
                         (x[unusedU5])) {
                y = (x[unusedU5] + x[vlen]) / 2.0;
              } else {
                y = x[unusedU5] + (x[vlen] - x[unusedU5]) / 2.0;
              }
            } else if (((x[k] < 0.0) != (x[unusedU5] < 0.0)) || rtIsInf(x[k])) {
              y = (x[k] + x[unusedU5]) / 2.0;
            } else {
              y = x[k] + (x[unusedU5] - x[k]) / 2.0;
            }
          }
        } else {
          int midm1;
          midm1 = vlen >> 1;
          if ((vlen & 1) == 0) {
            unusedU3.set_size(x.size(0));
            k = x.size(0);
            for (unusedU5 = 0; unusedU5 < k; unusedU5++) {
              unusedU3[unusedU5] = x[unusedU5];
            }

            quickselect(unusedU3, midm1 + 1, vlen, &y, &k, &unusedU5);
            if (midm1 < k) {
              quickselect(unusedU3, midm1, unusedU5 - 1, &b, &k, &vlen);
              if (((y < 0.0) != (b < 0.0)) || rtIsInf(y)) {
                y = (y + b) / 2.0;
              } else {
                y += (b - y) / 2.0;
              }
            }
          } else {
            unusedU3.set_size(x.size(0));
            k = x.size(0);
            for (unusedU5 = 0; unusedU5 < k; unusedU5++) {
              unusedU3[unusedU5] = x[unusedU5];
            }

            quickselect(unusedU3, midm1 + 1, vlen, &y, &k, &unusedU5);
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

// End of code generation (median.cpp)
