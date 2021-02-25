//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  postEstimateClustersCPP.cpp
//
//  Code generation for function 'postEstimateClustersCPP'
//


// Include files
#include "postEstimateClustersCPP.h"
#include "minOrMax.h"
#include "postEstimateClustersCPP_data.h"
#include "postEstimateClustersCPP_initialize.h"
#include "rt_nonfinite.h"

// Function Definitions
void postEstimateClustersCPP(const coder::array<double, 1U> &x_norm, const coder::
  array<double, 1U> &y_norm, const coder::array<double, 1U> &TTC, coder::array<
  int, 1U> &idx, coder::array<double, 2U> &boxes, coder::array<double, 1U> &TTCs)
{
  int b_idx;
  int trueCount;
  int i;
  coder::array<double, 1U> x_norm2;
  int n;
  coder::array<double, 1U> y_norm2;
  coder::array<double, 1U> TTC2;
  int maxval_tmp;
  int k;
  coder::array<double, 1U> clus_xmin;
  int b_trueCount;
  coder::array<double, 1U> clus_ymin;
  coder::array<double, 1U> clus_xmax;
  coder::array<double, 1U> clean_x;
  coder::array<double, 1U> clean_y;
  double ex;
  coder::array<int, 1U> r;
  if (!isInitialized_postEstimateClustersCPP) {
    postEstimateClustersCPP_initialize();
  }

  //  Keep only values assigned to clusters
  b_idx = idx.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      trueCount++;
    }
  }

  x_norm2.set_size(trueCount);
  n = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      x_norm2[n] = x_norm[i];
      n++;
    }
  }

  b_idx = idx.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      trueCount++;
    }
  }

  y_norm2.set_size(trueCount);
  n = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      y_norm2[n] = y_norm[i];
      n++;
    }
  }


  b_idx = idx.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      trueCount++;
    }
  }

  TTC2.set_size(trueCount);
  n = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      TTC2[n] = TTC[i];
      n++;
    }
  }

  b_idx = idx.size(0) - 1;
  trueCount = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      trueCount++;
    }
  }

  n = 0;
  for (i = 0; i <= b_idx; i++) {
    if (idx[i] > 0) {
      idx[n] = idx[i];
      n++;
    }
  }

  idx.set_size(trueCount);

  //  Calculate final cluster parameters,
  //  calculate object FoE, and check if FoE is inside of bounding box
  maxval_tmp = idx[0];
  for (k = 2; k <= trueCount; k++) {
    b_trueCount = idx[k - 1];
    if (maxval_tmp < b_trueCount) {
      maxval_tmp = b_trueCount;
    }
  }

  clus_xmin.set_size(maxval_tmp);
  clus_ymin.set_size(maxval_tmp);
  clus_xmax.set_size(maxval_tmp);
  for (b_trueCount = 0; b_trueCount < maxval_tmp; b_trueCount++) {
    clus_xmin[b_trueCount] = 0.0;
    clus_ymin[b_trueCount] = 0.0;
    clus_xmax[b_trueCount] = 0.0;
  }

  boxes.set_size(maxval_tmp, 4);
  b_idx = maxval_tmp << 2;
  for (b_trueCount = 0; b_trueCount < b_idx; b_trueCount++) {
    boxes[b_trueCount] = 0.0;
  }

  TTCs.set_size(maxval_tmp);
  for (b_trueCount = 0; b_trueCount < maxval_tmp; b_trueCount++) {
    TTCs[b_trueCount] = 0.0;
  }

  for (k = 0; k < maxval_tmp; k++) {
    int end;
    boolean_T exitg1;
    double d;

    b_idx = trueCount - 1;
    b_trueCount = 0;
    for (i = 0; i <= b_idx; i++) {
      if (idx[i] == k + 1) {
        b_trueCount++;
      }
    }

    clean_x.set_size(b_trueCount);
    n = 0;

    end = trueCount - 1;
    b_trueCount = 0;
    for (i = 0; i <= b_idx; i++) {
      if (idx[i] == k + 1) {
        clean_x[n] = x_norm2[i];
        n++;
      }

      if (idx[i] == k + 1) {
        b_trueCount++;
      }
    }

    clean_y.set_size(b_trueCount);
    n = 0;
    for (i = 0; i <= end; i++) {
      if (idx[i] == k + 1) {
        clean_y[n] = y_norm2[i];
        n++;
      }
    }

    clus_xmin[k] = minimum(clean_x) * 240.0;
    clus_ymin[k] = minimum(clean_y) * 180.0;
    n = clean_x.size(0);
    if (clean_x.size(0) <= 2) {
      if (clean_x.size(0) == 1) {
        ex = clean_x[0];
      } else if ((clean_x[0] < clean_x[1]) || (rtIsNaN(clean_x[0]) && (!rtIsNaN
                   (clean_x[1])))) {
        ex = clean_x[1];
      } else {
        ex = clean_x[0];
      }
    } else {
      if (!rtIsNaN(clean_x[0])) {
        b_idx = 1;
      } else {
        b_idx = 0;
        end = 2;
        exitg1 = false;
        while ((!exitg1) && (end <= clean_x.size(0))) {
          if (!rtIsNaN(clean_x[end - 1])) {
            b_idx = end;
            exitg1 = true;
          } else {
            end++;
          }
        }
      }

      if (b_idx == 0) {
        ex = clean_x[0];
      } else {
        ex = clean_x[b_idx - 1];
        b_trueCount = b_idx + 1;
        for (end = b_trueCount; end <= n; end++) {
          d = clean_x[end - 1];
          if (ex < d) {
            ex = d;
          }
        }
      }
    }

    clus_xmax[k] = ex * 240.0;
    n = clean_y.size(0);
    if (clean_y.size(0) <= 2) {
      if (clean_y.size(0) == 1) {
        ex = clean_y[0];
      } else if ((clean_y[0] < clean_y[1]) || (rtIsNaN(clean_y[0]) && (!rtIsNaN
                   (clean_y[1])))) {
        ex = clean_y[1];
      } else {
        ex = clean_y[0];
      }
    } else {
      if (!rtIsNaN(clean_y[0])) {
        b_idx = 1;
      } else {
        b_idx = 0;
        end = 2;
        exitg1 = false;
        while ((!exitg1) && (end <= clean_y.size(0))) {
          if (!rtIsNaN(clean_y[end - 1])) {
            b_idx = end;
            exitg1 = true;
          } else {
            end++;
          }
        }
      }

      if (b_idx == 0) {
        ex = clean_y[0];
      } else {
        ex = clean_y[b_idx - 1];
        b_trueCount = b_idx + 1;
        for (end = b_trueCount; end <= n; end++) {
          d = clean_y[end - 1];
          if (ex < d) {
            ex = d;
          }
        }
      }
    }

    boxes[k] = clus_xmin[k];
    boxes[k + boxes.size(0)] = clus_ymin[k];
    boxes[k + boxes.size(0) * 2] = clus_xmax[k] - clus_xmin[k];
    boxes[k + boxes.size(0) * 3] = ex * 180.0 - clus_ymin[k];


    b_idx = trueCount - 1;
    b_trueCount = 0;
    for (i = 0; i <= b_idx; i++) {
      if (idx[i] == k + 1) {
        b_trueCount++;
      }
    }

    r.set_size(b_trueCount);
    n = 0;
    for (i = 0; i <= b_idx; i++) {
      if (idx[i] == k + 1) {
        r[n] = i + 1;
        n++;
      }
    }

    b_trueCount = r.size(0);
    if (r.size(0) == 0) {
      ex = 0.0;
    } else {
      ex = TTC2[r[0] - 1];
      for (end = 2; end <= b_trueCount; end++) {
        ex += TTC2[r[end - 1] - 1];
      }
    }

    TTCs[k] = ex / static_cast<double>(r.size(0)) / 100.0;

  }
}

// End of code generation (postEstimateClustersCPP.cpp)
