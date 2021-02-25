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
#include "prepEstimateClustersCPP.h"
#include "median.h"
#include "prepEstimateClustersCPP_data.h"
#include "prepEstimateClustersCPP_initialize.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "std.h"
#include <cmath>
#include <math.h>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void prepEstimateClustersCPP(const coder::array<double, 2U> &optic_flow, double
  FoE_x_estimate, coder::array<double, 1U> &x_norm, coder::array<double, 1U>
  &y_norm, coder::array<double, 1U> &u_norm, coder::array<double, 1U> &v_norm,
  coder::array<double, 1U> &TTC, coder::array<double, 1U> &ang_norm, coder::
  array<double, 1U> &TTC_norm)
{
  int nx;
  coder::array<double, 1U> xs;
  int k;
  int trueCount;
  int i;
  coder::array<int, 1U> r;
  coder::array<double, 1U> mag_OF;
  coder::array<int, 1U> r1;
  coder::array<double, 1U> y;
  double d;
  double b_y;
  coder::array<boolean_T, 1U> idx_in;
  int b_trueCount;
  int c_trueCount;
  if (!isInitialized_prepEstimateClustersCPP) {
    prepEstimateClustersCPP_initialize();
  }

  //  Prep data for CPP clustering
  nx = optic_flow.size(0);
  xs.set_size(optic_flow.size(0));
  for (k = 0; k < nx; k++) {
    xs[k] = optic_flow[k];
  }

  nx = optic_flow.size(0);
  y_norm.set_size(optic_flow.size(0));
  for (k = 0; k < nx; k++) {
    y_norm[k] = optic_flow[k + optic_flow.size(0)];
  }

  nx = optic_flow.size(0);
  u_norm.set_size(optic_flow.size(0));
  for (k = 0; k < nx; k++) {
    u_norm[k] = optic_flow[k + optic_flow.size(0) * 2];
  }

  nx = optic_flow.size(0);
  v_norm.set_size(optic_flow.size(0));
  for (k = 0; k < nx; k++) {
    v_norm[k] = optic_flow[k + optic_flow.size(0) * 3];
  }

  //  Calculate vector angles and map all: -pi/2 to pi/2
  ang_norm.set_size(optic_flow.size(0));
  nx = optic_flow.size(0);
  for (k = 0; k < nx; k++) {
    ang_norm[k] = rt_atan2d_snf(optic_flow[k + optic_flow.size(0) * 3],
      optic_flow[k + optic_flow.size(0) * 2]);
  }

  nx = ang_norm.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= nx; i++) {
    if (ang_norm[i] > 1.5707963267948966) {
      trueCount++;
    }
  }

  r.set_size(trueCount);
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (ang_norm[i] > 1.5707963267948966) {
      r[k] = i + 1;
      k++;
    }
  }

  mag_OF.set_size(r.size(0));
  nx = r.size(0);
  for (k = 0; k < nx; k++) {
    mag_OF[k] = ang_norm[r[k] - 1] - 3.1415926535897931;
  }

  nx = mag_OF.size(0);
  for (k = 0; k < nx; k++) {
    ang_norm[r[k] - 1] = mag_OF[k];
  }

  nx = ang_norm.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= nx; i++) {
    if (ang_norm[i] < -1.5707963267948966) {
      trueCount++;
    }
  }

  r1.set_size(trueCount);
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (ang_norm[i] < -1.5707963267948966) {
      r1[k] = i + 1;
      k++;
    }
  }

  mag_OF.set_size(r1.size(0));
  nx = r1.size(0);
  for (k = 0; k < nx; k++) {
    mag_OF[k] = ang_norm[r1[k] - 1] + 3.1415926535897931;
  }

  nx = mag_OF.size(0);
  for (k = 0; k < nx; k++) {
    ang_norm[r1[k] - 1] = mag_OF[k];
  }

  //  Calculate OF magnitude
  mag_OF.set_size(optic_flow.size(0));
  nx = optic_flow.size(0);
  for (k = 0; k < nx; k++) {
    d = optic_flow[k + optic_flow.size(0) * 2];
    mag_OF[k] = d * d;
  }

  y.set_size(optic_flow.size(0));
  nx = optic_flow.size(0);
  for (k = 0; k < nx; k++) {
    d = optic_flow[k + optic_flow.size(0) * 3];
    y[k] = d * d;
  }

  nx = mag_OF.size(0);
  for (k = 0; k < nx; k++) {
    mag_OF[k] = mag_OF[k] + y[k];
  }

  nx = mag_OF.size(0);
  for (k = 0; k < nx; k++) {
    mag_OF[k] = std::sqrt(mag_OF[k]);
  }

  //  Filter out high magnitude outliers
  nx = mag_OF.size(0);
  if (mag_OF.size(0) == 0) {
    b_y = 0.0;
  } else {
    b_y = mag_OF[0];
    for (k = 2; k <= nx; k++) {
      b_y += mag_OF[k - 1];
    }
  }

  b_y = b_y / static_cast<double>(mag_OF.size(0)) + 2.0 * b_std(mag_OF);
  idx_in.set_size(mag_OF.size(0));
  nx = mag_OF.size(0);
  for (k = 0; k < nx; k++) {
    idx_in[k] = (mag_OF[k] < b_y);
  }

  nx = idx_in.size(0) - 1;
  trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      trueCount++;
      mag_OF[k] = mag_OF[i];
      k++;
    }
  }

  mag_OF.set_size(trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      ang_norm[k] = ang_norm[i];
      k++;
    }
  }

  ang_norm.set_size(b_trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      xs[k] = xs[i];
      k++;
    }
  }

  xs.set_size(b_trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      y_norm[k] = y_norm[i];
      k++;
    }
  }

  y_norm.set_size(b_trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      u_norm[k] = u_norm[i];
      k++;
    }
  }

  u_norm.set_size(b_trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      v_norm[k] = v_norm[i];
      k++;
    }
  }

  v_norm.set_size(b_trueCount);

  //  Only keep section of high OF magnitudes,
  d = median(mag_OF) * 0.0;
  idx_in.set_size(trueCount);
  for (k = 0; k < trueCount; k++) {
    idx_in[k] = (mag_OF[k] > d);
  }

  nx = idx_in.size(0) - 1;
  trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      trueCount++;
      xs[k] = xs[i];
      k++;
    }
  }

  xs.set_size(trueCount);
  nx = idx_in.size(0) - 1;
  b_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      b_trueCount++;
      y_norm[k] = y_norm[i];
      k++;
    }
  }

  y_norm.set_size(b_trueCount);
  nx = idx_in.size(0) - 1;
  c_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      c_trueCount++;
      u_norm[k] = u_norm[i];
      k++;
    }
  }

  u_norm.set_size(c_trueCount);
  nx = idx_in.size(0) - 1;
  c_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      c_trueCount++;
      v_norm[k] = v_norm[i];
      k++;
    }
  }

  v_norm.set_size(c_trueCount);
  nx = idx_in.size(0) - 1;
  c_trueCount = 0;
  k = 0;
  for (i = 0; i <= nx; i++) {
    if (idx_in[i]) {
      c_trueCount++;
      ang_norm[k] = ang_norm[i];
      k++;
    }
  }

  ang_norm.set_size(c_trueCount);

  //  normalize variables to [0, 1]
  x_norm.set_size(trueCount);
  for (k = 0; k < trueCount; k++) {
    x_norm[k] = xs[k] / 240.0;
  }

  for (k = 0; k < b_trueCount; k++) {
    y_norm[k] = y_norm[k] / 180.0;
  }

  //  normalize angle, magnitude and u around mean with 1 stdev
  if (c_trueCount == 0) {
    b_y = 0.0;
  } else {
    b_y = ang_norm[0];
    for (k = 2; k <= c_trueCount; k++) {
      b_y += ang_norm[k - 1];
    }
  }

  b_y /= static_cast<double>(c_trueCount);
  d = b_std(ang_norm);
  for (k = 0; k < c_trueCount; k++) {
    ang_norm[k] = (ang_norm[k] - b_y) / d;
  }

  for (k = 0; k < trueCount; k++) {
    xs[k] = (xs[k] - FoE_x_estimate) / u_norm[k];
  }

  nx = xs.size(0);
  TTC.set_size(xs.size(0));
  for (k = 0; k < nx; k++) {
    TTC[k] = std::abs(xs[k]);
  }

  nx = TTC.size(0);
  for (i = 0; i < nx; i++) {
    if (TTC[i] > 3000.0) {
      TTC[i] = 3000.0;
    }
  }

  nx = TTC.size(0);
  if (TTC.size(0) == 0) {
    b_y = 0.0;
  } else {
    b_y = TTC[0];
    for (k = 2; k <= nx; k++) {
      b_y += TTC[k - 1];
    }
  }

  b_y /= static_cast<double>(TTC.size(0));
  d = b_std(TTC);
  TTC_norm.set_size(TTC.size(0));
  nx = TTC.size(0);
  for (k = 0; k < nx; k++) {
    TTC_norm[k] = (TTC[k] - b_y) / d;
  }
}

// End of code generation (prepEstimateClustersCPP.cpp)
