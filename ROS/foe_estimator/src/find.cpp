//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  find.cpp
//
//  Code generation for function 'find'
//


// Include files
#include "find.h"
#include "estimateFoECPP.h"
#include "rt_nonfinite.h"

// Function Definitions
void b_eml_find(const boolean_T x[1000], int i_data[], int i_size[1])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 1000)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 1000) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (1 > idx) {
    i_size[0] = 0;
  } else {
    i_size[0] = idx;
  }
}

void c_eml_find(const coder::array<boolean_T, 1U> &x, coder::array<int, 1U> &i)
{
  int nx;
  int idx;
  int ii;
  boolean_T exitg1;
  nx = x.size(0);
  idx = 0;
  i.set_size(x.size(0));
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx - 1)) {
    if (x[ii]) {
      idx++;
      i[idx - 1] = ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }

  if (1 > idx) {
    idx = 0;
  }

  i.set_size(idx);
}

void eml_find(const boolean_T x[1000], int i_data[], int i_size[1])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size[0] = 1;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 1000)) {
    if (x[ii]) {
      idx = 1;
      i_data[0] = ii + 1;
      exitg1 = true;
    } else {
      ii++;
    }
  }

  if (idx == 0) {
    i_size[0] = 0;
  }
}

// End of code generation (find.cpp)
