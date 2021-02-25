//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  postEstimateClustersCPP.h
//
//  Code generation for function 'postEstimateClustersCPP'
//


#ifndef POSTESTIMATECLUSTERSCPP_H
#define POSTESTIMATECLUSTERSCPP_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "postEstimateClustersCPP_types.h"

// Function Declarations
extern void postEstimateClustersCPP(const coder::array<double, 1U> &x_norm,
  const coder::array<double, 1U> &y_norm, const coder::array<double, 1U> &TTC,
  coder::array<int, 1U> &idx, coder::array<double, 2U> &boxes, coder::array<
  double, 1U> &TTCs);

#endif

// End of code generation (postEstimateClustersCPP.h)
