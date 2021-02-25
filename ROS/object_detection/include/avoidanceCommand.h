//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  avoidanceCommand.h
//
//  Code generation for function 'avoidanceCommand'
//


#ifndef AVOIDANCECOMMAND_H
#define AVOIDANCECOMMAND_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "avoidanceCommand_types.h"

// Function Declarations
extern void avoidanceCommand(const coder::array<double, 2U> &clus_bbox, const
  coder::array<double, 1U> &FoE_info, double FoE_x_estimate, double *col_timer,
  boolean_T *rolling, double *roll_command);

#endif

// End of code generation (avoidanceCommand.h)
