//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  postEstimateClustersCPP_initialize.cpp
//
//  Code generation for function 'postEstimateClustersCPP_initialize'
//


// Include files
#include "postEstimateClustersCPP_initialize.h"
#include "postEstimateClustersCPP.h"
#include "postEstimateClustersCPP_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void postEstimateClustersCPP_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_postEstimateClustersCPP = true;
}

// End of code generation (postEstimateClustersCPP_initialize.cpp)
