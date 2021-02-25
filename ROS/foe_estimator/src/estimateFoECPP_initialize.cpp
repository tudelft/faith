//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  estimateFoECPP_initialize.cpp
//
//  Code generation for function 'estimateFoECPP_initialize'
//


// Include files
#include "estimateFoECPP_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "estimateFoECPP.h"
#include "estimateFoECPP_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void estimateFoECPP_initialize()
{
  rt_InitInfAndNaN();
  c_eml_rand_mt19937ar_stateful_i();
  isInitialized_estimateFoECPP = true;
}

// End of code generation (estimateFoECPP_initialize.cpp)
