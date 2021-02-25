//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  avoidanceCommand.cpp
//
//  Code generation for function 'avoidanceCommand'
//


// Include files
#include "avoidanceCommand.h"
#include "avoidanceCommand_data.h"
#include "avoidanceCommand_initialize.h"
#include "rt_nonfinite.h"

// Function Definitions
void avoidanceCommand(const coder::array<double, 2U> &clus_bbox, const coder::
                      array<double, 1U> &FoE_info, double FoE_x_estimate, double
                      *col_timer, boolean_T *rolling, double *roll_command)
{
  int n;
  int idx;
  double ex;
  int k;
  int iindx;
  boolean_T exitg1;
  int i;
  double d;
  if (!isInitialized_avoidanceCommand) {
    avoidanceCommand_initialize();
  }

  n = FoE_info.size(0);
  if (FoE_info.size(0) <= 2) {
    if (FoE_info.size(0) == 1) {
      ex = FoE_info[0];
      iindx = 0;
    } else if ((FoE_info[0] > FoE_info[1]) || (rtIsNaN(FoE_info[0]) && (!rtIsNaN
                 (FoE_info[1])))) {
      ex = FoE_info[1];
      iindx = 1;
    } else {
      ex = FoE_info[0];
      iindx = 0;
    }
  } else {
    if (!rtIsNaN(FoE_info[0])) {
      idx = 1;
    } else {
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= FoE_info.size(0))) {
        if (!rtIsNaN(FoE_info[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      ex = FoE_info[0];
      iindx = 0;
    } else {
      ex = FoE_info[idx - 1];
      iindx = idx - 1;
      i = idx + 1;
      for (k = i; k <= n; k++) {
        d = FoE_info[k - 1];
        if (ex > d) {
          ex = d;
          iindx = k - 1;
        }
      }
    }
  }

  //  Weak assumption collision check (with col_buff percent buffer)
  if ((FoE_x_estimate > clus_bbox[iindx] - clus_bbox[iindx + clus_bbox.size(0) *
       2] * 1.0) && (FoE_x_estimate < (clus_bbox[iindx] + clus_bbox[iindx +
        clus_bbox.size(0) * 2]) + clus_bbox[iindx + clus_bbox.size(0) * 2] * 1.0))
  {
    if (ex < 7.0) {
      (*col_timer)++;
    }
  } else {
    *col_timer = 0.0;
  }

  //  Avoidance manouvre to 'free' space
  if (*col_timer > 1.0) {
    n = FoE_info.size(0);
    if (FoE_info.size(0) <= 2) {
      if (FoE_info.size(0) == 1) {
        idx = 1;
      } else if ((FoE_info[0] < FoE_info[1]) || (rtIsNaN(FoE_info[0]) &&
                  (!rtIsNaN(FoE_info[1])))) {
        idx = 2;
      } else {
        idx = 1;
      }
    } else {
      if (!rtIsNaN(FoE_info[0])) {
        idx = 1;
      } else {
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= FoE_info.size(0))) {
          if (!rtIsNaN(FoE_info[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        idx = 1;
      } else {
        ex = FoE_info[idx - 1];
        i = idx + 1;
        for (k = i; k <= n; k++) {
          d = FoE_info[k - 1];
          if (ex < d) {
            ex = d;
            idx = k;
          }
        }
      }
    }

    if (!*rolling) {
      *roll_command = clus_bbox[idx - 1] - clus_bbox[iindx];
      if (*roll_command < 0.0) {
        *roll_command = -1.0;
      } else if (*roll_command > 0.0) {
        *roll_command = 1.0;
      } else {
        if (*roll_command == 0.0) {
          *roll_command = 0.0;
        }
      }
    }

    *rolling = true;

  } else {
    *rolling = false;
    *roll_command = 0.0;
  }
}

// End of code generation (avoidanceCommand.cpp)
