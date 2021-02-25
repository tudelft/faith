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

#include "rt_nonfinite.h"
#include <cmath>
#include <limits>

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

// Function: rt_InitInfAndNaN ==================================================
//  Abstract:
//  Initialize the rtInf, rtMinusInf, and rtNaN needed by the
//  generated code. NaN is initialized as non-signaling. Assumes IEEE.

void rt_InitInfAndNaN()
{
  rtNaN = std::numeric_limits<real_T>::quiet_NaN();
  rtNaNF = std::numeric_limits<real32_T>::quiet_NaN();
  rtInf = std::numeric_limits<real_T>::infinity();
  rtInfF = std::numeric_limits<real32_T>::infinity();
  rtMinusInf = -std::numeric_limits<real_T>::infinity();
  rtMinusInfF = -std::numeric_limits<real32_T>::infinity();
}

// Function: rtIsInf ==================================================
//  Abstract:
//  Test if value is infinite

boolean_T rtIsInf(real_T value)
{
  return ((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

// Function: rtIsInfF =================================================
//  Abstract:
//  Test if single-precision value is infinite

boolean_T rtIsInfF(real32_T value)
{
  return(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

// Function: rtIsNaN ==================================================
//  Abstract:
//  Test if value is not a number

boolean_T rtIsNaN(real_T value)
{
  return ((value!=value)? 1U : 0U);
}

// Function: rtIsNaNF =================================================
//  Abstract:
//  Test if single-precision value is not a number

boolean_T rtIsNaNF(real32_T value)
{
  return ((value!=value)? 1U : 0U);
}

// End of code generation (rt_nonfinite.cpp)
