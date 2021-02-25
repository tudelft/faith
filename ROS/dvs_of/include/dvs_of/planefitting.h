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

#ifndef PLANEFITTING_H_
#define PLANEFITTING_H_

#include "dvs_of/calibration.h"
#include "dvs_of/settings.h"

namespace dvs_of {

    enum flow_error_flag{
    NO_ERROR,
    REFRACTORY_PERIOD,
    TOO_CLOSE_TO_EDGE,
    RATE_LIMITED,
    TOO_FEW_EVENTS1,
    NO_TIME_GRADIENT,
    TOO_FEW_EVENTS2,
    TOO_LARGE_NRMSE,
    TOO_FEW_EVENTS3,
    DET_TOO_SMALL,
    MAG_TOO_LARGE,
    NUM_ERRORS
    };
    
    struct IMU {
        double_t acc_x;
        double_t acc_y;
        double_t acc_z;
        double_t gyr_x;
        double_t gyr_y;
        double_t gyr_z;
        uint64_t t;
    };

    /**
     * Raw event structure
     *      (x,y)   pixel coordinates, with (0,0) being the top left corner
     *      p       pixel polarity (0 or 1)
     *      t       event's timestamp in microseconds
     */
    struct Events {
        uint16_t x;
        uint16_t y;
        uint16_t p;
        uint64_t t;
    };

    /**
     * Flow packet
     *      (x,y)   pixel coordinates
     *      (ux,uy) undistorted pixel coordinates
     *      p       polarity
     *      t       timestamp (no 64 bits)
     *      (u,v)   horizontal and vertical optic flow speeds
     */
    struct FlowPacket {
        uint16_t x;
        uint16_t y;
        uint64_t t;
        uint16_t p;

        float u;
        float v;

        float ux;
        float uy;

        int64_t tP;
        float NMSE;

        bool valid;
    };

    /**
     * States and parameters of the optic flow algorithm (plane fitting)
     *      r           radius of the surrounding pixels in the neighborhood (2*r+1)
     *      kernelSize  number of pixels within the neighborhood (2*r+1)^2
     *      minPixels   minimum number of pixels in the neighborhood
     *      *dxKernel   will contain all the neighboring pixels' x coordinate
     *      *dyKernel   will contain all the neighboring pixels' y coordinate
     *      refPeriod   refractory period
     */
    struct FlowState {
        uint16_t r;
        std::size_t kernelSize;
        uint32_t nReject;
        uint16_t minPixels;
        int16_t *dxKernel;
        int16_t *dyKernel;
        
        int64_t dtMax;
        int64_t refPeriod;

        bool  limitEventRate;   // Hz
        int64_t minDt;          // us
        float rateSetpoint;
        float rateTimeConstant;

        float maxNRMSE;
        float dtStopFactor;

        float vMax;

        float flowRate;
        uint64_t lastEventT;
    };

    /** 
     *
     */
    struct point3d {
    uint16_t xx, yy;
    int64_t dt;
    bool isUsed;
    };

    /**
     * Buffer of events
     *      bufferPosition  current position inside buffer
     *      bufferUsedSize  size of data currently inside buffer, in bytes
     *      bufferSize      size of buffer, in bytes
     *      sizeX           
     *      sizeY           
     */
    struct simple2DBufferLong {
        size_t bufferPosition;
        size_t bufferUsedSize;
        size_t bufferSize;

        uint16_t sizeX = DIMX;
        uint16_t sizeY = DIMY;

        uint64_t buffer2D[DIMX][DIMY];
    };

    /**
     * Prototypes
     */
    void freeFlowStateMem(FlowState *myFlowState);
    void determinePixelNeighborhood(FlowState *myFlowState);
    bool simple2DBufferLongSet(simple2DBufferLong buffer, uint16_t x, uint16_t y, uint64_t value);
    enum flow_error_flag computeOpticFlow(FlowPacket *fp, simple2DBufferLong *buffer, FlowState *myFlowState);
    void flowAdaptiveUpdateRate(int64_t lastFlowDt, FlowState *myFlowState);

} // namespace

#endif // PLANEFITTING_H_