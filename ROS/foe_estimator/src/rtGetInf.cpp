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

#include "rtGetInf.h"

// Function: rtGetInf ==================================================================
//  Abstract:
//  Initialize rtInf needed by the generated code.

real_T rtGetInf(void)
{
  return rtInf;
}

// Function: rtGetInfF =================================================================
//  Abstract:
//  Initialize rtInfF needed by the generated code.

real32_T rtGetInfF(void)
{
  return rtInfF;
}

// Function: rtGetMinusInf =============================================================
//  Abstract:
//  Initialize rtMinusInf needed by the generated code.

real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

// Function: rtGetMinusInfF ============================================================
//  Abstract:
//  Initialize rtMinusInfF needed by the generated code.

real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

// End of code generation (rtGetInf.cpp)
