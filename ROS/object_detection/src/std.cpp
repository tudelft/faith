/**
 * This file is part of the object_detection package - MAVLab TU Delft
 * 
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 * 
 * */

// Include files
#include "std.h"
#include "prepEstimateClustersCPP.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
double b_std(const coder::array<double, 1U> &x)
{
  double y;
  int kend;
  double xbar;
  coder::array<double, 1U> absdiff;
  kend = x.size(0);
  if (x.size(0) == 0) {
    y = rtNaN;
  } else if (x.size(0) == 1) {
    if ((!rtIsInf(x[0])) && (!rtIsNaN(x[0]))) {
      y = 0.0;
    } else {
      y = rtNaN;
    }
  } else {
    int k;
    xbar = x[0];
    for (k = 2; k <= kend; k++) {
      xbar += x[k - 1];
    }

    xbar /= static_cast<double>(x.size(0));
    absdiff.set_size(x.size(0));
    for (k = 0; k < kend; k++) {
      absdiff[k] = std::abs(x[k] - xbar);
    }

    y = 0.0;
    xbar = 3.3121686421112381E-170;
    kend = x.size(0);
    for (k = 0; k < kend; k++) {
      if (absdiff[k] > xbar) {
        double t;
        t = xbar / absdiff[k];
        y = y * t * t + 1.0;
        xbar = absdiff[k];
      } else {
        double t;
        t = absdiff[k] / xbar;
        y += t * t;
      }
    }

    y = xbar * std::sqrt(y);
    y /= std::sqrt(static_cast<double>(x.size(0)) - 1.0);
  }

  return y;
}

// End of code generation (std.cpp)
