/**
 * This file is part of the foe_estimator package - MAVLab TU Delft
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
