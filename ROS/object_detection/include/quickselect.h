//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  quickselect.h
//
//  Code generation for function 'quickselect'
//


#ifndef QUICKSELECT_H
#define QUICKSELECT_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "prepEstimateClustersCPP_types.h"

// Function Declarations
extern void quickselect(coder::array<double, 1U> &v, int n, int vlen, double *vn,
  int *nfirst, int *nlast);

#endif

// End of code generation (quickselect.h)
