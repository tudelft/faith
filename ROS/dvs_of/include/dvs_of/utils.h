/**
 * This file is part of the dvs_of package - MAVLab TU Delft
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

#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <ctime>
#include <cmath>
#include <cstring>
#include <climits>
#include <csignal>
#include "dvs_of/timer.h"

/**
 * Convert boolean to string
 */
inline const char * const BoolToString(bool b){
  return b ? "TRUE" : "FALSE";
}

/**
 * Rates struct
 */
struct rates_t {
    float p;
    float q;
    float r;
    int64_t ts;
};

/**
 * Optic flow message
 */
struct __attribute__((aligned)) flow_msg {
    int16_t xu;
    int16_t yu;
    int16_t u;
    int16_t v;
};

/**
 * Bound float to int16
 */
inline int16_t float2int16(float val) {
    int16_t ret;
    if (val > SHRT_MAX) {
        ret = SHRT_MAX;
    } else if (val < SHRT_MIN) {
        ret = SHRT_MIN;
    } else {
        ret = (int16_t)val;
    }
    return ret;
}

/**
 * Get current date/time, format is YYYY-MM-DD.HH:mm:ss
 */
inline const std::string currentDateTime(void) {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y_%m_%d-%H_%M_%S", &tstruct);
    return buf;
}

/** 
 * Center element of an array
 */
inline float array_middle(float *array, int size) {
    if (size % 2)
        return array[size >> 1];
    return (array[size / 2] + array[size / 2 - 1]) / 2;
}

/**
 * Median of an array
 */
inline float median(float *data, int size) {
    float *sortData = new float[size];
    float temp;
    int i, j; 
    std::memcpy(sortData, data, sizeof(float) * size);
    // Insertion Sort
    for (i = 1; i < size; i++) {
        temp = sortData[i];
        j = i - 1;
        while (j >= 0 && temp < sortData[j]) {
            sortData[j + 1] = sortData[j];
            j = j - 1;
        }
        sortData[j + 1] = temp;
    }
    // Return data value in middle of sorted array
    float median = array_middle(sortData, size);
    free(sortData);
    return median;
}

/**
 * Median filter
 */
class MedianFilter {
    private:
        float *data, *sortData;
        uint8_t dataIndex;
        uint8_t size;
    public:

        // Constructor
        MedianFilter(uint8_t num_elm) {
            if (num_elm % 2 == 0) {
                num_elm += 1;
            }
            data = new float[num_elm];
            sortData = new float[num_elm];
            dataIndex = 0;
            size = num_elm;
        }
        
        // Destructor
        ~MedianFilter(void) {
            if (data != NULL) {
                delete[] data;
                delete[] sortData;
            }
        }

        // Get size
        uint8_t get_size(){
            return size;
        }

        // Get median of array
        float get_median() {
            return array_middle(sortData, size);
        }

        float update(float new_data) {
            float temp;
            int i, j;
            data[dataIndex] = new_data;
            dataIndex = (dataIndex + 1) % size;
            std::memcpy(sortData, data, sizeof(float) * size);
            for (i = 1; i < size; i++) {
                temp = sortData[i];
                j = i - 1;
                while (j >= 0 && temp < sortData[j]) {
                    sortData[j + 1] = sortData[j];
                    j = j - 1;
                }
                sortData[j + 1] = temp;
            }
            return get_median();
        }
};


#endif /* INCLUDE_UTILS_H_ */