/**
 * This file is part of the object_detection package - MAVLab TU Delft
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

#pragma once
#include <thread>
#include <math.h>
#include <ros/ros.h>

#include <mutex>

#include <iostream>
#include <fstream>

#include "std_msgs/Int32.h"

// Required by clustering algorithm 
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "prepEstimateClustersCPP_types.h"
#include "prepEstimateClustersCPP.h"
#include "prepEstimateClustersCPP_terminate.h"

#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "postEstimateClustersCPP_types.h"
#include "postEstimateClustersCPP.h"
#include "postEstimateClustersCPP_terminate.h"

#include "avoidanceCommand_types.h"
#include "avoidanceCommand.h"
#include "avoidanceCommand_terminate.h"

// DBSCAN header 
#include "dbscan.h"
#include <vector>



class Controller {
    private:
        std::thread control_job_; 

    public:
        int cnt;

        // Initialize counters 
        int64_t last_ts = 0;
        double prev_roll = 0;

        Controller();
        ~Controller();

        void control_job();
        void clusterServer();
        
        // DBSCAN data struct
        struct vec2f {
            float data[2];
            float operator[](int idx) const { return data[idx]; }
        };

        // Array for storing OF (COLUMN MAJOR)
        coder::array<double, 2U> fillOpticFlowArray();
        std::vector<Controller::vec2f> fillDBSCANdata();

        // MATLAB array for storing cluster Idx
        coder::array<int, 1U> fillIdx(std::vector<std::vector<uint>> clusters);


        
        struct FlowPacket {
        uint16_t x;
        uint16_t y;
        uint64_t t;
        float u;
        float v;
        };

        // Record clustering
        std::ofstream Cluster_rec_file;

        // Prep arrays for detection function
        coder::array<double, 2U> optic_flow;
        coder::array<double, 1U> x_norm;
        coder::array<double, 1U> y_norm;
        coder::array<double, 1U> u_norm;
        coder::array<double, 1U> v_norm;
        coder::array<double, 1U> TTC;
        coder::array<double, 1U> ang_norm;
        coder::array<double, 1U> TTC_norm;

        coder::array<int, 1U> idx;
        coder::array<double, 2U> boxes;
        coder::array<double, 1U> TTCs;


        void log_OF(std::vector<Controller::FlowPacket> *myOF);
        std::vector<Controller::FlowPacket> final_buffer;
        std::vector<Controller::FlowPacket> myOF;
        std::vector<Controller::FlowPacket> myOFBuf;

        // Mutex to lock during looping through arrays
        std::mutex prepMutex;

        // FoE message
        std_msgs::Int32 FoE_msg;
        int32_t FoE_x;

        ros::Publisher roll_pub;
        std_msgs::Int32 roll_msg;

        double roll_command = 0;
        double col_timer = 0;
        boolean_T rolling  = false;


};
