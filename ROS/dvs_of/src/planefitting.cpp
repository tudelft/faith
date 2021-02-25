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

#include <dvs_of/planefitting.h>

namespace dvs_of{

    /**
     * Get interface to cAER's simple2DBufferLong type
     */
    static inline uint64_t simple2DBufferLongGet(simple2DBufferLong buffer, uint16_t x, uint16_t y){
        if (x >= buffer.sizeX || y >= buffer.sizeY)
            return 0;
        return buffer.buffer2D[x][y];
    }

    /** 
     * Sort point3d in assending order with negative values always larger than positive values
     */
    int comparePoints(const void *a, const void *b){
        if (((point3d *)a)->dt <  0)
            return 1;
        if (((point3d *)a)->dt < ((point3d *)b)->dt)
            return -1;
        if (((point3d *)a)->dt == ((point3d *)b)->dt)
            return 0;
        return 1;
    }

    /**
     * Compute the optic flow based on the plane fitting algorithm 
     */
    enum flow_error_flag computeOpticFlow(FlowPacket *fp, simple2DBufferLong *buffer, FlowState *myFlowState){
        fp->valid = false;

        // Do not compute flow for points at edges
        if (fp->x < myFlowState->r || fp->x >= buffer->sizeX - myFlowState->r
        || fp->y < myFlowState->r || fp->y >= buffer->sizeY - myFlowState->r)
            return TOO_CLOSE_TO_EDGE;

        // Do not compute flow if the flow event rate is too high
        if (myFlowState->limitEventRate && fp->t < myFlowState->lastEventT + myFlowState->minDt) {
            std::cout << "fp->t = " << fp->t << "\t " << myFlowState->lastEventT + myFlowState->minDt << std::endl;
            return RATE_LIMITED;
        }

        // log time here to better regulate computation rate
        myFlowState->lastEventT = fp->t;

        // Add event to history
        if (!(fp->x >= buffer->sizeX || fp->y >= buffer->sizeY))
            buffer->buffer2D[fp->x][fp->y] = fp->t;

        point3d points[myFlowState->kernelSize]; 
        // Accumulate event timestamps
        for (uint32_t i = 0; i < myFlowState->kernelSize; i++) {
            points[i].xx = fp->x + myFlowState->dxKernel[i];
            points[i].yy = fp->y + myFlowState->dyKernel[i];
            points[i].dt = fp->t - buffer->buffer2D[points[i].xx][points[i].yy];
        }

        // Sort by timestamp, ascending
        qsort(points, myFlowState->kernelSize, sizeof(point3d), comparePoints);

        // Remove outliers
        uint32_t n = myFlowState->kernelSize;
        for (int32_t i = n - 1; i >= 0; i--) {
            if (points[i].dt < myFlowState->dtMax && points[i].dt >= 0)
                break;
            n--;
        }

        // Check for the number of events
        if (n < myFlowState->minPixels)
            return TOO_FEW_EVENTS1;

        // Event selection: where to cut off?
        int32_t dx1, dy1;
        int64_t dtMax = 0;
        bool found_first_dist = false;
        bool found_dt = false;
        for (uint32_t i = 0; i < n; i++) {
            // skip centre event
            if (points[i].xx == fp->x && points[i].yy == fp->y) {
                continue;
            }
            if (!found_first_dist) {
                // set first distance
                dx1 = (int32_t)points[i].xx - (int32_t)fp->x;
                dy1 = (int32_t)points[i].yy - (int32_t)fp->y;
                found_first_dist = true;
            } else if (!found_dt) {
                // Set dtMax at first linearly independent pair of points
                if (points[i].dt > 0 && ((int32_t)points[i].xx - (int32_t)fp->x) * dy1 - ((int32_t)points[i].yy - (int32_t)fp->y) * dx1 != 0) {
                    dtMax = (int64_t)(points[i].dt * myFlowState->dtStopFactor);
                    found_dt = true;
                }
            } else if (points[i].dt - points[i - 1].dt > dtMax) {
                n = i;
                break;
            }
        }

        // If no gradient in time (max dt = 0), probably due to a flash (or overflow), ignore.
        if(dtMax <= 0)        
            return NO_TIME_GRADIENT;

        // Check for the number of events
        if (n < myFlowState->minPixels)
            return TOO_FEW_EVENTS2; 

        // Now compute flow statistics
        float sx2 = 0;
        float sy2 = 0;
        float st2 = 0;
        float sxy = 0;
        float sxt = 0;
        float syt = 0;
        float st = 0;

        float xU = dvsGetUndistortedPixelX(fp->x, fp->y);
        float yU = dvsGetUndistortedPixelY(fp->x, fp->y);

        float dxus[myFlowState->kernelSize];
        float dyus[myFlowState->kernelSize];
        float dtus[myFlowState->kernelSize];

        for (uint32_t i = 0; i < n; i++) {
            points[i].isUsed = true;
            float xxU = dvsGetUndistortedPixelX(points[i].xx, points[i].yy);
            float yyU = dvsGetUndistortedPixelY(points[i].xx, points[i].yy);
            dxus[i] = xxU - xU;
            dyus[i] = yyU - yU;
            dtus[i] = -points[i].dt / SECONDS_TO_MICROSECONDS;
            sx2 += dxus[i] * dxus[i];
            sy2 += dyus[i] * dyus[i];
            st2 += dtus[i] * dtus[i];
            sxy += dxus[i] * dyus[i];
            sxt += dxus[i] * dtus[i];
            syt += dyus[i] * dtus[i];
            st  += dtus[i];
        }

        // Compute determinant and check invertibility
        float D = -sxy * sxy + sx2 * sy2;
        if (fabsf(D) < FLT_ZERO_EPSILON)
            return DET_TOO_SMALL;

        // Compute plane parameters
        float a = (sy2 * sxt - sxy * syt) / D;
        float b = (sx2 * syt - sxy * sxt) / D;

        // Compute R2
        float SSR = st2 - a * sxt - b * syt;
        float NMSE = SSR * (float) n / (st * st + FLT_ZERO_EPSILON); 

        // Reject outliers (if allowed)
        if (NMSE > myFlowState->maxNRMSE * myFlowState->maxNRMSE && REJECT_OUTLIERS == 1) {
            bool reject = true;
            uint32_t nInit = n;
            for (size_t nReject = 0; nReject < myFlowState->nReject; nReject++) {
                // Find point with max distance
                float dMax = 0; uint32_t iS = 0;
                for (uint32_t i = 0; i < nInit; i++) {
                    if (points[i].isUsed) {
                    float dist = fabsf(a * dxus[i] + b * dyus[i] - dtus[i]);
                    if (dist > dMax) {
                        iS = i;
                        dMax = dist;
                    }
                    }
                }
                sx2 -= dxus[iS] * dxus[iS];
                sy2 -= dyus[iS] * dyus[iS];
                st2 -= dtus[iS] * dtus[iS];
                sxy -= dxus[iS] * dyus[iS];
                sxt -= dxus[iS] * dtus[iS];
                syt -= dyus[iS] * dtus[iS];
                st  -= dtus[iS];
                points[iS].isUsed = false;
                n--;
                // Compute determinant and check invertibility
                D = -sxy * sxy + sx2 * sy2;
                if (fabsf(D) < FLT_ZERO_EPSILON) 
                    return DET_TOO_SMALL;
                // Compute plane parameters
                a = (sy2 * sxt - sxy * syt) / D;
                b = (sx2 * syt - sxy * sxt) / D;
                // Compute R2
                SSR = st2 - a * sxt - b * syt;
                NMSE = SSR * (float)n / (st * st + FLT_ZERO_EPSILON); 
                if (NMSE <= myFlowState->maxNRMSE * myFlowState->maxNRMSE) {
                    reject = false;
                    break;
                }
            }
            if (reject)
                return TOO_LARGE_NRMSE;
        }

        // Check for the number of events
        if (n < myFlowState->minPixels)
            return TOO_FEW_EVENTS3; 

        // Check for NaN value
        float plane_power = a * a + b * b;
        if (fabsf(plane_power) < FLT_ZERO_EPSILON)
            return DET_TOO_SMALL;

        // Compute velocity
        float scaleFactor = 1.0f / plane_power;
        float u = scaleFactor * a;
        float v = scaleFactor * b;

        // Reject if magnitude is too large
        if (u * u + v * v > myFlowState->vMax * myFlowState->vMax)
            return MAG_TOO_LARGE;

        uint32_t last_used_point = n - 1;
        while (!points[last_used_point].isUsed && last_used_point > 0)
            last_used_point--;

        // Assign flow to event in per second (instead of pix/us)
        fp->u = u;
        fp->v = v;
        fp->ux = xU;
        fp->uy = yU;
        fp->NMSE = NMSE;
        fp->valid = true;
        fp->tP = (int64_t)(fp->t) - points[last_used_point].dt;

        return NO_ERROR;
    }

    /**
     * Free flow state
     */
    void freeFlowStateMem(FlowState *myFlowState){
        delete myFlowState->dxKernel;
        delete myFlowState->dyKernel;
    }

} // namespace