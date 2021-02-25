/**
 * This file is part of the odroid_ros_dvs package - MAVLab TU Delft
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

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <memory>
#include <mutex>

#include <csignal>
#include <ctime>

#include <climits>
#include <cmath>
#include <vector>
#include <algorithm>

#define ARGS            4
#define DISPLAY         0    // 0: no display in terminal
#define ON_OFF_PROC     true // true: on-line

#define X_FOV_RADIUS    120
#define Y_FOV_RADIUS    90 // max 90

#define DIMX            240
#define DIMY            180

#define REJECT_OUTLIERS 1

#define MAX_COUNTER     200 // equivalent to 1s of measurements (IMU)

#define MAX_FLOW_MSGS 4096

#define M_PI 3.14159265358979323846
#define RadOfDeg(x) ((x) * (M_PI/180.))
#define SECONDS_TO_MICROSECONDS 1e6f
#define FLT_ZERO_EPSILON 1e-8f  // important this stays at -8