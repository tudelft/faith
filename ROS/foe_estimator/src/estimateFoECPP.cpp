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
#include "estimateFoECPP.h"
#include "det.h"
#include "estimateFoECPP_data.h"
#include "estimateFoECPP_initialize.h"
#include "find.h"
#include "minOrMax.h"
#include "mldivide.h"
#include "rand.h"
#include "rt_nonfinite.h"
#include "sign.h"
#include <cmath>
#include <cstring>

// Function Definitions
void estimateFoECPP(const coder::array<double, 2U> &optic_flow, double *FoE_x,
                    double *FoE_y)
{
  int loop_ub;
  coder::array<double, 1U> A;
  int i;
  double inter_x;
  coder::array<double, 1U> B;
  coder::array<double, 2U> norm_lines;
  int vlen;
  boolean_T empty_non_axis_sizes;
  signed char input_sizes_idx_1;
  signed char b_input_sizes_idx_1;
  signed char sizes_idx_1;
  coder::array<double, 2U> all_lines;
  int i1;
  int input_sizes_idx_0;
  int result_idx_0;
  coder::array<double, 2U> all_rules;
  coder::array<boolean_T, 1U> b_B;
  static const short varargin_2[20] = { 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, -180, -240,
    -1, 0, 1, 0, 0, -1, 0, 1 };

  int trueCount;
  int b_i;
  coder::array<int, 1U> r;
  int partialTrueCount;
  double max_score;
  double new_rule_idx;
  static const short init_hull_lines[12] = { 0, 1, 0, 1, 1, 0, 1, 0, 0, 0, -180,
    -240 };

  int b_trueCount;
  double best_hull_p_data[2000];
  int nxtfree_data[1];
  coder::array<signed char, 1U> vec_tried;
  double filled_idx_data[1000];
  int all_points_size[1];
  double all_points_data[1000];
  double all_points[2000];
  signed char point_in_hull[1000];
  static const unsigned char init_hull_points[8] = { 0U, 0U, 240U, 240U, 0U,
    180U, 180U, 0U };

  short tmp_data[1000];
  short b_tmp_data[1000];
  short c_tmp_data[1000];
  short d_tmp_data[1000];
  coder::array<int, 1U> r1;
  short e_tmp_data[1000];
  coder::array<int, 1U> r2;
  double A_merge[4];
  double b_all_lines[2];
  signed char new_point_in_hull[1000];
  double intersect_xy[2];
  double cur_points_data[2000];
  short f_tmp_data[1000];
  coder::array<int, 1U> r3;
  coder::array<signed char, 1U> new_vec_in_hull;
  coder::array<int, 1U> r4;
  coder::array<int, 1U> r5;
  boolean_T b_point_in_hull[1000];
  coder::array<int, 1U> r6;
  int g_tmp_data[1000];
  coder::array<int, 1U> r7;
  short h_tmp_data[1000];
  if (!isInitialized_estimateFoECPP) {
    estimateFoECPP_initialize();
  }

  //  Set method parameters
  //  Set # iterations for RANSAC search
  //  Set # failures allowed in iteration before stopping search
  //  Initialize frame boundaries to hull
  //  Rules: [A, B, C, sign(v), sign(u)]
  //  Initialize optical flow variables
  //  Write into ax + by + c = 0
  loop_ub = optic_flow.size(0);
  A.set_size(optic_flow.size(0));
  for (i = 0; i < loop_ub; i++) {
    inter_x = optic_flow[i + optic_flow.size(0)];
    A[i] = inter_x - (inter_x + optic_flow[i + optic_flow.size(0) * 3]);
  }

  loop_ub = optic_flow.size(0);
  B.set_size(optic_flow.size(0));
  for (i = 0; i < loop_ub; i++) {
    inter_x = optic_flow[i];
    B[i] = (inter_x + optic_flow[i + optic_flow.size(0) * 2]) - inter_x;
  }

  //  C = (x1.*y2 - x2.*y1);
  //  In format ax + by + c = 0
  norm_lines.set_size(B.size(0), 3);
  loop_ub = B.size(0);
  for (i = 0; i < loop_ub; i++) {
    norm_lines[i] = B[i];
  }

  loop_ub = A.size(0);
  for (i = 0; i < loop_ub; i++) {
    norm_lines[i + norm_lines.size(0)] = -A[i];
  }

  loop_ub = B.size(0);
  for (i = 0; i < loop_ub; i++) {
    norm_lines[i + norm_lines.size(0) * 2] = -(B[i] * optic_flow[i] - A[i] *
      optic_flow[i + optic_flow.size(0)]);
  }

  //  Create a list with the rules for all vectors
  //  Rule: [A, B, C, sign(v), sign(u)]
  loop_ub = optic_flow.size(0);
  A.set_size(optic_flow.size(0));
  for (i = 0; i < loop_ub; i++) {
    A[i] = optic_flow[i + optic_flow.size(0) * 3];
  }

  b_sign(A);
  loop_ub = optic_flow.size(0);
  B.set_size(optic_flow.size(0));
  for (i = 0; i < loop_ub; i++) {
    B[i] = optic_flow[i + optic_flow.size(0) * 2];
  }

  b_sign(B);
  if (norm_lines.size(0) != 0) {
    vlen = norm_lines.size(0);
  } else if (A.size(0) != 0) {
    vlen = A.size(0);
  } else if (B.size(0) != 0) {
    vlen = B.size(0);
  } else {
    vlen = 0;
  }

  empty_non_axis_sizes = (vlen == 0);
  if (empty_non_axis_sizes || (norm_lines.size(0) != 0)) {
    input_sizes_idx_1 = 3;
  } else {
    input_sizes_idx_1 = 0;
  }

  if (empty_non_axis_sizes || (A.size(0) != 0)) {
    b_input_sizes_idx_1 = 1;
  } else {
    b_input_sizes_idx_1 = 0;
  }

  if (empty_non_axis_sizes || (B.size(0) != 0)) {
    sizes_idx_1 = 1;
  } else {
    sizes_idx_1 = 0;
  }

  all_lines.set_size(vlen, ((input_sizes_idx_1 + b_input_sizes_idx_1) +
    sizes_idx_1));
  loop_ub = input_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < vlen; i1++) {
      all_lines[i1 + all_lines.size(0) * i] = norm_lines[i1 + norm_lines.size(0)
        * i];
    }
  }

  loop_ub = b_input_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < vlen; i1++) {
      all_lines[i1 + all_lines.size(0) * input_sizes_idx_1] = A[i1];
    }
  }

  loop_ub = sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    for (i1 = 0; i1 < vlen; i1++) {
      all_lines[i1 + all_lines.size(0) * (input_sizes_idx_1 +
        b_input_sizes_idx_1)] = B[i1];
    }
  }

  if ((all_lines.size(0) != 0) && (all_lines.size(1) != 0)) {
    input_sizes_idx_0 = all_lines.size(0);
    result_idx_0 = all_lines.size(0);
  } else {
    input_sizes_idx_0 = 0;
    result_idx_0 = 0;
  }

  all_rules.set_size((input_sizes_idx_0 + 4), 5);
  for (i = 0; i < 5; i++) {
    for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
      all_rules[i1 + all_rules.size(0) * i] = all_lines[i1 + all_lines.size(0) *
        i];
    }

    vlen = i << 2;
    all_rules[result_idx_0 + all_rules.size(0) * i] = varargin_2[vlen];
    all_rules[(result_idx_0 + all_rules.size(0) * i) + 1] = varargin_2[vlen + 1];
    all_rules[(result_idx_0 + all_rules.size(0) * i) + 2] = varargin_2[vlen + 2];
    all_rules[(result_idx_0 + all_rules.size(0) * i) + 3] = varargin_2[vlen + 3];
  }

  //  Put line descriptions in same format for halfplane check
  loop_ub = all_rules.size(0);
  b_B.set_size(all_rules.size(0));
  for (i = 0; i < loop_ub; i++) {
    b_B[i] = (all_rules[i] < 0.0);
  }

  input_sizes_idx_0 = b_B.size(0) - 1;
  trueCount = 0;
  for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
    if (b_B[b_i]) {
      trueCount++;
    }
  }

  r.set_size(trueCount);
  partialTrueCount = 0;
  for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
    if (b_B[b_i]) {
      r[partialTrueCount] = b_i + 1;
      partialTrueCount++;
    }
  }

  all_lines.set_size(r.size(0), 3);
  loop_ub = r.size(0);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      all_lines[i1 + all_lines.size(0) * i] = -all_rules[(r[i1] + all_rules.size
        (0) * i) - 1];
    }
  }

  loop_ub = all_lines.size(0);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      all_rules[(r[i1] + all_rules.size(0) * i) - 1] = all_lines[i1 +
        all_lines.size(0) * i];
    }
  }

  all_lines.set_size((norm_lines.size(0) + 4), 3);
  loop_ub = norm_lines.size(0);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      all_lines[i1 + all_lines.size(0) * i] = norm_lines[i1 + norm_lines.size(0)
        * i];
    }

    vlen = i << 2;
    all_lines[norm_lines.size(0) + all_lines.size(0) * i] = init_hull_lines[vlen];
    all_lines[(norm_lines.size(0) + all_lines.size(0) * i) + 1] =
      init_hull_lines[vlen + 1];
    all_lines[(norm_lines.size(0) + all_lines.size(0) * i) + 2] =
      init_hull_lines[vlen + 2];
    all_lines[(norm_lines.size(0) + all_lines.size(0) * i) + 3] =
      init_hull_lines[vlen + 3];
  }

  //  Initialize score bookkeeping
  max_score = 0.0;

  //  Initialize for c++ code
  new_rule_idx = 0.0;
  b_trueCount = 1;
  best_hull_p_data[0] = 0.0;
  best_hull_p_data[1] = 0.0;
  loop_ub = all_rules.size(0);
  i = all_rules.size(0) - 1;
  for (int n = 0; n < 50; n++) {
    double stop_cnt;
    int c_trueCount;
    int d_trueCount;
    int e_trueCount;
    double x_it_center;
    double y_it_center;
    double vec_N_idx_2;
    double inlier_cnt;
    double u;
    double illegal_cnt;
    boolean_T guard1 = false;
    B.set_size(loop_ub);
    vec_tried.set_size(all_rules.size(0));
    for (i1 = 0; i1 < loop_ub; i1++) {
      B[i1] = 0.0;
      vec_tried[i1] = 0;
    }

    B[all_rules.size(0) - 4] = 1.0;
    vec_tried[all_rules.size(0) - 4] = 1;
    B[all_rules.size(0) - 3] = 1.0;
    vec_tried[all_rules.size(0) - 3] = 1;
    B[all_rules.size(0) - 2] = 1.0;
    vec_tried[all_rules.size(0) - 2] = 1;
    B[all_rules.size(0) - 1] = 1.0;
    vec_tried[all_rules.size(0) - 1] = 1;
    std::memset(&all_points[0], 0, 2000U * sizeof(double));
    for (i1 = 0; i1 < 2; i1++) {
      vlen = i1 << 2;
      all_points[1000 * i1] = init_hull_points[vlen];
      all_points[1000 * i1 + 1] = init_hull_points[vlen + 1];
      all_points[1000 * i1 + 2] = init_hull_points[vlen + 2];
      all_points[1000 * i1 + 3] = init_hull_points[vlen + 3];
    }

    std::memset(&point_in_hull[0], 0, 1000U * sizeof(signed char));
    point_in_hull[0] = 1;
    point_in_hull[1] = 1;
    point_in_hull[2] = 1;
    point_in_hull[3] = 1;
    empty_non_axis_sizes = false;

    //  Keep adding a randomly chosen vector to the hull
    //  Stop if new vector violates all hull rules
    stop_cnt = 0.0;
    while (!empty_non_axis_sizes) {
      double rand_N;
      boolean_T add_check;

      //  Keep adding a randomly chosen vector to the hull, stop if new vector
      //  Stop if new vector violates all hull rules
      rand_N = b_rand();
      rand_N = std::floor(rand_N * static_cast<double>(norm_lines.size(0))) +
        1.0;
      int exitg1;
      do {
        exitg1 = 0;
        i1 = static_cast<int>(rand_N) - 1;
        if ((B[i1] == 1.0) || (vec_tried[i1] == 1)) {
          vlen = B.size(0);
          inter_x = B[0];
          for (input_sizes_idx_0 = 2; input_sizes_idx_0 <= vlen;
               input_sizes_idx_0++) {
            inter_x += B[input_sizes_idx_0 - 1];
          }

          if (inter_x < B.size(0)) {
            rand_N = b_rand();
            rand_N = std::floor(rand_N * static_cast<double>(norm_lines.size(0)))
              + 1.0;
          } else {
            exitg1 = 1;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      vec_tried[i1] = 1;

      //  Stop also if all vectors belong to hull, or all vectors
      //  have been tried
      vlen = B.size(0);
      inter_x = B[0];
      for (input_sizes_idx_0 = 2; input_sizes_idx_0 <= vlen; input_sizes_idx_0++)
      {
        inter_x += B[input_sizes_idx_0 - 1];
      }

      if (inter_x == B.size(0)) {
        empty_non_axis_sizes = true;
      } else {
        vlen = vec_tried.size(0);
        inter_x = vec_tried[0];
        for (input_sizes_idx_0 = 2; input_sizes_idx_0 <= vlen; input_sizes_idx_0
             ++) {
          inter_x += static_cast<double>(vec_tried[input_sizes_idx_0 - 1]);
        }

        if (inter_x == vec_tried.size(0)) {
          empty_non_axis_sizes = true;
        }
      }

      x_it_center = norm_lines[i1];
      y_it_center = norm_lines[i1 + norm_lines.size(0)];
      vec_N_idx_2 = norm_lines[i1 + norm_lines.size(0) * 2];

      //  Rule: [A, B, C, sign(v), sign(u), vec_idx]
      //          vec_N_rule = [vec_N,sign(v(rand_N)), sign(u(rand_N)), rand_N]; 
      //  Put line descriptions in same format for halfplane check
      if (x_it_center < 0.0) {
        x_it_center = -x_it_center;
        y_it_center = -y_it_center;
        vec_N_idx_2 = -vec_N_idx_2;

      }

      //         %% Calculate all intersections and check points against hull rules 
      add_check = false;
      illegal_cnt = 0.0;
      input_sizes_idx_0 = B.size(0) - 1;
      trueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          trueCount++;
        }
      }

      r1.set_size(trueCount);
      partialTrueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          r1[partialTrueCount] = b_i + 1;
          partialTrueCount++;
        }
      }

      //  Temp matrix for inserting in A_merge and B_merge
      input_sizes_idx_0 = B.size(0) - 1;
      trueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          trueCount++;
        }
      }

      r2.set_size(trueCount);
      partialTrueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          r2[partialTrueCount] = b_i + 1;
          partialTrueCount++;
        }
      }

      e_trueCount = r2.size(0);
      if (0 <= r2.size(0) - 1) {
        A_merge[1] = x_it_center;
        A_merge[3] = y_it_center;
      }

      for (vlen = 0; vlen < e_trueCount; vlen++) {
        A_merge[0] = all_lines[r1[vlen] - 1];
        A_merge[2] = all_lines[(r1[vlen] + all_lines.size(0)) - 1];

        //  Skip if lines are parallel (necessary for A inversion)
        if (std::abs(det(A_merge)) > 2.2204460492503131E-12) {
          boolean_T illegal;

          //  calculate intersection of lines
          b_all_lines[0] = -all_lines[(r1[vlen] + all_lines.size(0) * 2) - 1];
          b_all_lines[1] = -vec_N_idx_2;
          mldivide(A_merge, b_all_lines, intersect_xy);
          inter_x = intersect_xy[0];
          x_it_center = intersect_xy[1];

          //  Check if intersection adheres to hull rules
          illegal = false;
          input_sizes_idx_0 = B.size(0) - 1;
          trueCount = 0;
          for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
            if (B[b_i] == 1.0) {
              trueCount++;
            }
          }

          r3.set_size(trueCount);
          partialTrueCount = 0;
          for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
            if (B[b_i] == 1.0) {
              r3[partialTrueCount] = b_i + 1;
              partialTrueCount++;
            }
          }

          d_trueCount = r3.size(0);
          for (result_idx_0 = 0; result_idx_0 < d_trueCount; result_idx_0++) {
            input_sizes_idx_0 = B.size(0) - 1;
            trueCount = 0;
            for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
              if (B[b_i] == 1.0) {
                trueCount++;
              }
            }

            r4.set_size(trueCount);
            partialTrueCount = 0;
            for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
              if (B[b_i] == 1.0) {
                r4[partialTrueCount] = b_i + 1;
                partialTrueCount++;
              }
            }

            y_it_center = all_rules[(r4[result_idx_0] + all_rules.size(0)) - 1];
            u = y_it_center;
            if (y_it_center < 0.0) {
              u = -1.0;
            } else if (y_it_center > 0.0) {
              u = 1.0;
            } else {
              if (y_it_center == 0.0) {
                u = 0.0;
              }
            }

            guard1 = false;
            if (((u == 1.0) && (all_rules[(r4[result_idx_0] + all_rules.size(0) *
                   3) - 1] == 1.0)) || ((u == -1.0) && (all_rules
                  [(r4[result_idx_0] + all_rules.size(0) * 3) - 1] == -1.0)) ||
                ((u == 0.0) && (all_rules[(r4[result_idx_0] + all_rules.size(0) *
                   4) - 1] == 1.0))) {
              guard1 = true;
            } else {
              inlier_cnt = all_rules[r4[result_idx_0] - 1];
              if (inlier_cnt < 0.0) {
                inlier_cnt = -1.0;
              } else if (inlier_cnt > 0.0) {
                inlier_cnt = 1.0;
              } else {
                if (inlier_cnt == 0.0) {
                  inlier_cnt = 0.0;
                }
              }

              if ((inlier_cnt == 0.0) && (all_rules[(r4[result_idx_0] +
                    all_rules.size(0) * 3) - 1] == 1.0)) {
                guard1 = true;
              } else {
                if ((((u == -1.0) && (all_rules[(r4[result_idx_0] +
                        all_rules.size(0) * 3) - 1] == 1.0)) || ((u == 1.0) &&
                      (all_rules[(r4[result_idx_0] + all_rules.size(0) * 3) - 1]
                       == -1.0)) || ((u == 0.0) && (all_rules[(r4[result_idx_0]
                        + all_rules.size(0) * 4) - 1] == -1.0)) || ((inlier_cnt ==
                       0.0) && (all_rules[(r4[result_idx_0] + all_rules.size(0) *
                        3) - 1] == -1.0))) && (!((all_rules[r4[result_idx_0] - 1]
                       * inter_x + all_rules[(r4[result_idx_0] + all_rules.size
                        (0)) - 1] * x_it_center) + all_rules[(r4[result_idx_0] +
                       all_rules.size(0) * 2) - 1] >= -2.2204460492503131E-12)))
                {
                  illegal = true;
                  illegal_cnt++;
                }
              }
            }

            if (guard1 && (!((all_rules[r4[result_idx_0] - 1] * inter_x +
                              y_it_center * x_it_center) + all_rules
                             [(r4[result_idx_0] + all_rules.size(0) * 2) - 1] <=
                             2.2204460492503131E-12))) {
              illegal = true;
              illegal_cnt++;
            }
          }

          //                 Add vector to hull
          if ((!illegal) && (!add_check)) {

            new_rule_idx = rand_N;

            //  LATEST RULE,USE IN INTERSECTION REMOVAL
            B[i1] = 1.0;
            add_check = true;
          }

          //  Add intersection to hull
          if (!illegal) {
            for (d_trueCount = 0; d_trueCount < 1000; d_trueCount++) {
              b_point_in_hull[d_trueCount] = (point_in_hull[d_trueCount] == 0);
            }

            eml_find(b_point_in_hull, nxtfree_data, all_points_size);
            input_sizes_idx_0 = all_points_size[0];
            for (d_trueCount = 0; d_trueCount < input_sizes_idx_0; d_trueCount++)
            {
              all_points[nxtfree_data[d_trueCount] - 1] = intersect_xy[0];
            }

            input_sizes_idx_0 = all_points_size[0];
            for (d_trueCount = 0; d_trueCount < input_sizes_idx_0; d_trueCount++)
            {
              all_points[nxtfree_data[d_trueCount] + 999] = intersect_xy[1];
            }

            input_sizes_idx_0 = all_points_size[0];
            for (d_trueCount = 0; d_trueCount < input_sizes_idx_0; d_trueCount++)
            {
              point_in_hull[nxtfree_data[d_trueCount] - 1] = 1;
            }

            add_check = true;
          }

          //  Stop search if vector is not contributing towards smaller hull
          input_sizes_idx_0 = B.size(0) - 1;
          trueCount = 0;
          for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
            if (B[b_i] == 1.0) {
              trueCount++;
            }
          }

          r6.set_size(trueCount);
          partialTrueCount = 0;
          for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
            if (B[b_i] == 1.0) {
              r6[partialTrueCount] = b_i + 1;
              partialTrueCount++;
            }
          }

          if (illegal_cnt == r6.size(0)) {
            stop_cnt++;
            empty_non_axis_sizes = ((stop_cnt == 1.0) || empty_non_axis_sizes);
          }
        } else {
          //  Count parallel as illegal
          illegal_cnt++;
        }
      }

      //  Check new hull points & lines for hull conditions
      //  Create temp copies as reference for new arrays
      //  create temporary copy to use in selecting rule
      trueCount = 0;
      partialTrueCount = 0;
      for (b_i = 0; b_i < 1000; b_i++) {
        new_point_in_hull[b_i] = point_in_hull[b_i];
        if (point_in_hull[b_i] == 1) {
          trueCount++;
          f_tmp_data[partialTrueCount] = static_cast<short>(b_i + 1);
          partialTrueCount++;
        }
      }

      for (i1 = 0; i1 < trueCount; i1++) {
        cur_points_data[i1] = all_points[f_tmp_data[i1] - 1];
      }

      for (i1 = 0; i1 < trueCount; i1++) {
        cur_points_data[i1 + trueCount] = all_points[f_tmp_data[i1] + 999];
      }

      //  Tmp array for selecting point below
      c_trueCount = 0;
      for (b_i = 0; b_i < 1000; b_i++) {
        if (point_in_hull[b_i] == 1) {
          c_trueCount++;
        }
      }

      for (result_idx_0 = 0; result_idx_0 < c_trueCount; result_idx_0++) {
        // Select intersection from hull
        inter_x = cur_points_data[result_idx_0 + trueCount];

        //  Pick latest rule
        //  Check selected intersection against latest rule
        input_sizes_idx_0 = static_cast<int>(new_rule_idx) - 1;
        y_it_center = all_rules[input_sizes_idx_0 + all_rules.size(0)];
        u = y_it_center;
        if (y_it_center < 0.0) {
          u = -1.0;
        } else if (y_it_center > 0.0) {
          u = 1.0;
        } else {
          if (y_it_center == 0.0) {
            u = 0.0;
          }
        }

        guard1 = false;
        if (((u == 1.0) && (all_rules[input_sizes_idx_0 + all_rules.size(0) * 3]
                            == 1.0)) || ((u == -1.0) &&
             (all_rules[input_sizes_idx_0 + all_rules.size(0) * 3] == -1.0)) ||
            ((u == 0.0) && (all_rules[input_sizes_idx_0 + all_rules.size(0) * 4]
                            == 1.0))) {
          guard1 = true;
        } else {
          inlier_cnt = all_rules[input_sizes_idx_0];
          if (inlier_cnt < 0.0) {
            inlier_cnt = -1.0;
          } else if (inlier_cnt > 0.0) {
            inlier_cnt = 1.0;
          } else {
            if (inlier_cnt == 0.0) {
              inlier_cnt = 0.0;
            }
          }

          if ((inlier_cnt == 0.0) && (all_rules[input_sizes_idx_0 +
               all_rules.size(0) * 3] == 1.0)) {
            guard1 = true;
          } else {
            if ((((u == -1.0) && (all_rules[input_sizes_idx_0 + all_rules.size(0)
                                  * 3] == 1.0)) || ((u == 1.0) &&
                  (all_rules[input_sizes_idx_0 + all_rules.size(0) * 3] == -1.0))
                 || ((u == 0.0) && (all_rules[input_sizes_idx_0 + all_rules.size
                                    (0) * 4] == -1.0)) || ((inlier_cnt == 0.0) &&
                  (all_rules[input_sizes_idx_0 + all_rules.size(0) * 3] == -1.0)))
                && (!((all_rules[input_sizes_idx_0] *
                       cur_points_data[result_idx_0] +
                       all_rules[input_sizes_idx_0 + all_rules.size(0)] *
                       inter_x) + all_rules[input_sizes_idx_0 + all_rules.size(0)
                      * 2] >= -2.2204460492503131E-12))) {
              for (i1 = 0; i1 < 1000; i1++) {
                b_point_in_hull[i1] = (point_in_hull[i1] == 1);
              }

              b_eml_find(b_point_in_hull, g_tmp_data, nxtfree_data);
              input_sizes_idx_0 = nxtfree_data[0];
              for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
                filled_idx_data[i1] = g_tmp_data[i1];
              }

              input_sizes_idx_0 = static_cast<int>(filled_idx_data[result_idx_0])
                - 1;
              new_point_in_hull[input_sizes_idx_0] = 0;
              all_points[input_sizes_idx_0] = 0.0;
              all_points[static_cast<int>(filled_idx_data[result_idx_0]) + 999] =
                0.0;
            }
          }
        }

        if (guard1 && (!((all_rules[input_sizes_idx_0] *
                          cur_points_data[result_idx_0] + y_it_center * inter_x)
                         + all_rules[input_sizes_idx_0 + all_rules.size(0) * 2] <=
                         2.2204460492503131E-12))) {
          //  Set point_in_hull to negative
          for (i1 = 0; i1 < 1000; i1++) {
            b_point_in_hull[i1] = (point_in_hull[i1] == 1);
          }

          b_eml_find(b_point_in_hull, g_tmp_data, nxtfree_data);
          input_sizes_idx_0 = nxtfree_data[0];
          for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
            filled_idx_data[i1] = g_tmp_data[i1];
          }

          input_sizes_idx_0 = static_cast<int>(filled_idx_data[result_idx_0]) -
            1;
          new_point_in_hull[input_sizes_idx_0] = 0;
          all_points[input_sizes_idx_0] = 0.0;
          all_points[static_cast<int>(filled_idx_data[result_idx_0]) + 999] =
            0.0;
        }
      }

      std::memcpy(&point_in_hull[0], &new_point_in_hull[0], 1000U * sizeof
                  (signed char));

      //  Check hull lines for new hull conditions
      //  (hull points are already updated)
      new_vec_in_hull.set_size(B.size(0));
      input_sizes_idx_0 = B.size(0);
      for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
        new_vec_in_hull[i1] = static_cast<signed char>(B[i1]);
      }

      input_sizes_idx_0 = B.size(0) - 1;
      trueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          trueCount++;
        }
      }

      r5.set_size(trueCount);
      partialTrueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          r5[partialTrueCount] = b_i + 1;
          partialTrueCount++;
        }
      }

      //  Tmp rule array for checking below
      input_sizes_idx_0 = B.size(0) - 1;
      trueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          trueCount++;
        }
      }

      r7.set_size(trueCount);
      partialTrueCount = 0;
      for (b_i = 0; b_i <= input_sizes_idx_0; b_i++) {
        if (B[b_i] == 1.0) {
          r7[partialTrueCount] = b_i + 1;
          partialTrueCount++;
        }
      }

      i1 = r7.size(0);
      for (result_idx_0 = 0; result_idx_0 < i1; result_idx_0++) {
        trueCount = 0;
        partialTrueCount = 0;
        for (b_i = 0; b_i < 1000; b_i++) {
          if (new_point_in_hull[b_i] == 1) {
            trueCount++;
            h_tmp_data[partialTrueCount] = static_cast<short>(b_i + 1);
            partialTrueCount++;
          }
        }

        inter_x = all_rules[r5[result_idx_0] - 1];
        x_it_center = all_rules[(r5[result_idx_0] + all_rules.size(0)) - 1];
        y_it_center = all_rules[(r5[result_idx_0] + all_rules.size(0) * 2) - 1];
        for (e_trueCount = 0; e_trueCount < trueCount; e_trueCount++) {
          filled_idx_data[e_trueCount] = (inter_x *
            all_points[h_tmp_data[e_trueCount] - 1] + x_it_center *
            all_points[h_tmp_data[e_trueCount] + 999]) + y_it_center;
        }

        A.set_size(trueCount);
        for (e_trueCount = 0; e_trueCount < trueCount; e_trueCount++) {
          A[e_trueCount] = (filled_idx_data[e_trueCount] <
                            2.2204460492503131E-12) *
            (filled_idx_data[e_trueCount] > -2.2204460492503131E-12);
        }

        vlen = A.size(0);
        if (A.size(0) == 0) {
          inter_x = 0.0;
        } else {
          inter_x = A[0];
          for (input_sizes_idx_0 = 2; input_sizes_idx_0 <= vlen;
               input_sizes_idx_0++) {
            inter_x += A[input_sizes_idx_0 - 1];
          }
        }

        if (inter_x == 0.0) {
          //  Set the 'it' non-zero element from vec_in_hull to zero
          input_sizes_idx_0 = B.size(0);
          b_B.set_size(B.size(0));
          for (e_trueCount = 0; e_trueCount < input_sizes_idx_0; e_trueCount++)
          {
            b_B[e_trueCount] = (B[e_trueCount] != 0.0);
          }

          c_eml_find(b_B, r);
          A.set_size(r.size(0));
          input_sizes_idx_0 = r.size(0);
          for (e_trueCount = 0; e_trueCount < input_sizes_idx_0; e_trueCount++)
          {
            A[e_trueCount] = r[e_trueCount];
          }

          new_vec_in_hull[static_cast<int>(A[result_idx_0]) - 1] = 0;
        }
      }

      B.set_size(new_vec_in_hull.size(0));
      input_sizes_idx_0 = new_vec_in_hull.size(0);
      for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
        B[i1] = new_vec_in_hull[i1];
      }
    }

    //  Calculate score (= amount of correct halfplanes over all vectors for proposed FoE center) 
    trueCount = 0;
    partialTrueCount = 0;
    c_trueCount = 0;
    input_sizes_idx_0 = 0;
    d_trueCount = 0;
    result_idx_0 = 0;
    e_trueCount = 0;
    vlen = 0;
    for (b_i = 0; b_i < 1000; b_i++) {
      if (point_in_hull[b_i] == 1) {
        trueCount++;
        tmp_data[partialTrueCount] = static_cast<short>(b_i + 1);
        partialTrueCount++;
        c_trueCount++;
        b_tmp_data[input_sizes_idx_0] = static_cast<short>(b_i + 1);
        input_sizes_idx_0++;
        d_trueCount++;
        c_tmp_data[result_idx_0] = static_cast<short>(b_i + 1);
        result_idx_0++;
        e_trueCount++;
        d_tmp_data[vlen] = static_cast<short>(b_i + 1);
        vlen++;
      }
    }

    nxtfree_data[0] = c_trueCount;
    for (i1 = 0; i1 < c_trueCount; i1++) {
      filled_idx_data[i1] = all_points[b_tmp_data[i1] - 1];
    }

    all_points_size[0] = trueCount;
    for (i1 = 0; i1 < trueCount; i1++) {
      all_points_data[i1] = all_points[tmp_data[i1] - 1];
    }

    x_it_center = (maximum(filled_idx_data, nxtfree_data) + minimum
                   (all_points_data, all_points_size)) / 2.0;
    nxtfree_data[0] = e_trueCount;
    for (i1 = 0; i1 < e_trueCount; i1++) {
      filled_idx_data[i1] = all_points[d_tmp_data[i1] + 999];
    }

    all_points_size[0] = d_trueCount;
    for (i1 = 0; i1 < d_trueCount; i1++) {
      all_points_data[i1] = all_points[c_tmp_data[i1] + 999];
    }

    y_it_center = (maximum(filled_idx_data, nxtfree_data) + minimum
                   (all_points_data, all_points_size)) / 2.0;
    inlier_cnt = 0.0;
    for (vlen = 0; vlen <= i; vlen++) {
      inter_x = all_rules[vlen + all_rules.size(0)];
      u = inter_x;
      if (inter_x < 0.0) {
        u = -1.0;
      } else if (inter_x > 0.0) {
        u = 1.0;
      } else {
        if (inter_x == 0.0) {
          u = 0.0;
        }
      }

      guard1 = false;
      if (((u == 1.0) && (all_rules[vlen + all_rules.size(0) * 3] == 1.0)) ||
          ((u == -1.0) && (all_rules[vlen + all_rules.size(0) * 3] == -1.0)) ||
          ((u == 0.0) && (all_rules[vlen + all_rules.size(0) * 4] == 1.0))) {
        guard1 = true;
      } else {
        vec_N_idx_2 = all_rules[vlen];
        illegal_cnt = vec_N_idx_2;
        if (vec_N_idx_2 < 0.0) {
          illegal_cnt = -1.0;
        } else if (vec_N_idx_2 > 0.0) {
          illegal_cnt = 1.0;
        } else {
          if (vec_N_idx_2 == 0.0) {
            illegal_cnt = 0.0;
          }
        }

        if ((illegal_cnt == 0.0) && (all_rules[vlen + all_rules.size(0) * 3] ==
             1.0)) {
          guard1 = true;
        } else if ((((u == -1.0) && (all_rules[vlen + all_rules.size(0) * 3] ==
                      1.0)) || ((u == 1.0) && (all_rules[vlen + all_rules.size(0)
          * 3] == -1.0)) || ((u == 0.0) && (all_rules[vlen + all_rules.size(0) *
          4] == -1.0)) || ((illegal_cnt == 0.0) && (all_rules[vlen +
                      all_rules.size(0) * 3] == -1.0))) && ((vec_N_idx_2 *
                     x_it_center + inter_x * y_it_center) + all_rules[vlen +
                    all_rules.size(0) * 2] >= -2.2204460492503131E-12)) {
          inlier_cnt++;
        } else {
        }
      }

      if (guard1 && ((all_rules[vlen] * x_it_center + inter_x * y_it_center) +
                     all_rules[vlen + all_rules.size(0) * 2] <=
                     2.2204460492503131E-12)) {
        inlier_cnt++;
      } else {
      }
    }

    if (inlier_cnt > max_score) {
      max_score = inlier_cnt;
      b_trueCount = 0;
      partialTrueCount = 0;
      for (b_i = 0; b_i < 1000; b_i++) {
        if (point_in_hull[b_i] == 1) {
          b_trueCount++;
          e_tmp_data[partialTrueCount] = static_cast<short>(b_i + 1);
          partialTrueCount++;
        }
      }

      for (i1 = 0; i1 < b_trueCount; i1++) {
        best_hull_p_data[i1] = all_points[e_tmp_data[i1] - 1];
      }

      for (i1 = 0; i1 < b_trueCount; i1++) {
        best_hull_p_data[i1 + b_trueCount] = all_points[e_tmp_data[i1] + 999];
      }

    }

  }

  nxtfree_data[0] = b_trueCount;
  if (0 <= b_trueCount - 1) {
    std::memcpy(&filled_idx_data[0], &best_hull_p_data[0], b_trueCount * sizeof
                (double));
  }

  all_points_size[0] = b_trueCount;
  if (0 <= b_trueCount - 1) {
    std::memcpy(&all_points_data[0], &best_hull_p_data[0], b_trueCount * sizeof
                (double));
  }

  *FoE_x = (maximum(filled_idx_data, nxtfree_data) + minimum(all_points_data,
             all_points_size)) / 2.0;
  nxtfree_data[0] = b_trueCount;
  for (i = 0; i < b_trueCount; i++) {
    filled_idx_data[i] = best_hull_p_data[i + b_trueCount];
  }

  all_points_size[0] = b_trueCount;
  for (i = 0; i < b_trueCount; i++) {
    all_points_data[i] = best_hull_p_data[i + b_trueCount];
  }

  *FoE_y = (maximum(filled_idx_data, nxtfree_data) + minimum(all_points_data,
             all_points_size)) / 2.0;
}

// End of code generation (estimateFoECPP.cpp)
