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

#ifndef DVS_OF_H_
#define DVS_OF_H_

#include <string>
#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <dvs_of/FlowPacketMsg.h>
#include <dvs_of/FlowPacketMsgArray.h>

#include "dvs_of/planefitting.h"
#include "dvs_of/timer.h"
#include "dvs_of/utils.h"
#include "dvs_of/settings.h"

#include "std_msgs/Int32.h"


namespace dvs_of{

    /**
     * Rates class
     */
    class RateBuffer {
    private:
        int32_t length_;
        float period_;  // us
        uint8_t num_ele = 5;
        MedianFilter p_filt = MedianFilter(num_ele);
        MedianFilter q_filt = MedianFilter(num_ele);
        MedianFilter r_filt = MedianFilter(num_ele);

        rates_t *buffer_;
        int32_t buffer_idx_ = 0;

        std::mutex rates_mutex;

        uint32_t find_closest_rate_idx(int64_t ts);
    public:
        RateBuffer(float time_window, float rate);
        RateBuffer(float time_window, float rate, uint32_t SizeIMU);
        ~RateBuffer();

        int32_t getLength();
        void resetMedianFilters();

        // ON-Line processing of the incoming data
        void log_rates(std::vector<IMU> *myIMU);
        rates_t find_closest_rate(int64_t ts);
        rates_t find_average_rate(int64_t ts);

        // OFF-Line processing of the data-sets
        void offline_log_rates(std::vector<IMU> *myIMU);
    };

    /** 
     * Optic flow class
     */
    class OpticFlow {
        private:
            FILE *log_file;

            uint16_t x_offset;
            uint16_t y_offset;

            int32_t width_;
            int32_t height_;

            std::string FileName;

            simple2DBufferLong sae_on;
            simple2DBufferLong sae_off;

            float flow_errors[NUM_ERRORS];
            float total_events_ = 0.f;
            float total_flow_rejected_ = 0.f;
            float total_rotation_rejected_ = 0.f;
            float total_good_ = 0.f;
            float total_time_ = 0.f;
            clock_t start_time_;
            float peak_rate = 0;
            float min_comp_speed = 100000000;


        public:
            RateBuffer *myRates;            
            FlowState myFlowState;          
            FlowPacket myFlowPacket;        
            OpticFlow(int w, int h);
            OpticFlow(int w, int h, uint32_t SizeIMU);
            ~OpticFlow();
            void initFlowState();
            void determinePixelNeighborhood();
            void storeEventsFlow(float d_u, float d_v, float mag, rates_t rates, float rot_u, float rot_v, float chck);
            void computeOpticFlowVectors(std::vector<Events> *myEventsFOV, std::vector<IMU> *myIMU);
            void setLogFileName(std::string filename);

            int32_t FoE_x;
    };

    class Server {
        public:
            OpticFlow *myOpticFlow;            
            std::vector<Events> myEvents;
            std::vector<Events> myEventsFOV;
            std::vector<IMU>    myIMU;
            uint8_t mX = (uint8_t)(floor(DIMX/2));
            uint8_t mY = (uint8_t)(floor(DIMY/2));
            Server(ros::NodeHandle & nh, ros::NodeHandle nh_private);
            virtual ~Server();

            RateBuffer *insRateBuffer;

            void foeCallback(const std_msgs::Int32::ConstPtr& msg);

        private:
            ros::NodeHandle nh_;
            // Getting the Events
            void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
            // Getting the IMU data
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

            ros::Subscriber event_sub_;
            ros::Subscriber imu_sub_;
            ros::Publisher OF_pub_;
            ros::Subscriber foe_sub;
            
            std::ofstream DVS_rec_file;
            std::ofstream IMU_rec_file;
    };

} // namespace

#endif // DVS_OF_H_
