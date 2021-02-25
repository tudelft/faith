//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  find.h
//
//  Code generation for function 'find'
//


#ifndef FIND_H
#define FIND_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "estimateFoECPP_types.h"

// Function Declarations
extern void b_eml_find(const boolean_T x[1000], int i_data[], int i_size[1]);
extern void c_eml_find(const coder::array<boolean_T, 1U> &x, coder::array<int,
  1U> &i);
extern void eml_find(const boolean_T x[1000], int i_data[], int i_size[1]);

#endif

// End of code generation (find.h)
