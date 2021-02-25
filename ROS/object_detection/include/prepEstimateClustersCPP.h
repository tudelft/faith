//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  prepEstimateClustersCPP.h
//
//  Code generation for function 'prepEstimateClustersCPP'
//


#ifndef PREPESTIMATECLUSTERSCPP_H
#define PREPESTIMATECLUSTERSCPP_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "prepEstimateClustersCPP_types.h"

// Function Declarations
extern void prepEstimateClustersCPP(const coder::array<double, 2U> &optic_flow,
  double FoE_x_estimate, coder::array<double, 1U> &x_norm, coder::array<double,
  1U> &y_norm, coder::array<double, 1U> &u_norm, coder::array<double, 1U>
  &v_norm, coder::array<double, 1U> &TTC, coder::array<double, 1U> &ang_norm,
  coder::array<double, 1U> &TTC_norm);

#endif

// End of code generation (prepEstimateClustersCPP.h)
