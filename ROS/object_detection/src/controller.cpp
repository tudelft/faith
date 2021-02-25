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


#include "controller.hpp"

//============================================ MAIN ====================================================

coder::array<double, 2U> Controller::fillOpticFlowArray()
{
  coder::array<double, 2U> result;

  // Set the size of the array.
  int OFbuff_size = this->myOF.size();
  int buf_idx = 0;

  result.set_size(OFbuff_size, 4);
    
  this->prepMutex.lock();
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < (result.size(0)); idx0++) {

    // Fill the column major array    
    this->last_ts = this->myOF[idx0].t;
    result[idx0] = this->myOF[idx0].x * 1.0;
    result[idx0 + (result.size(0))] = this->myOF[idx0].y * 1.0;
    result[idx0 + 2*(result.size(0))] = this->myOF[idx0].u * 1.0;
    result[idx0 + 3*(result.size(0))] = this->myOF[idx0].v * 1.0;

    buf_idx++;
  }

  this->myOF.clear();
  this->prepMutex.unlock();

  return result;
}

// Fill the clustering vector
std::vector<Controller::vec2f> Controller::fillDBSCANdata()
{
  std::vector<Controller::vec2f> data;
  for (int cnt = 0; cnt < (this->x_norm.size(0)); cnt++) {
    Controller::vec2f tmp_data;
    
    float x_flt = this->x_norm[cnt];
    float TTC_flt = this->TTC_norm[cnt];
    
    tmp_data.data[0] = x_flt;
    tmp_data.data[1] = TTC_flt;

    data.push_back(tmp_data);
  } 

return data;
}

// Get cluster assignment per OF vector
coder::array<int, 1U> Controller::fillIdx(std::vector<std::vector<uint>> clusters)
{
  coder::array<int, 1U> result;

  // Set the size of the array.
  result.set_size(this->x_norm.size(0));

  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < result.size(0); idx0++) {
     result[idx0] = 0;
  }

  // fill result with cluster assignment 
  int vec_count = 0;
  for (int i = 0; i < clusters.size(); i++) {
    for (int j = 0; j < clusters[i].size(); j++) {
      int tmp_idx;
      tmp_idx = clusters[i][j];
      if (tmp_idx < this->x_norm.size(0)) {
        result[tmp_idx] = i+1;
      }
      vec_count++; 
    }
  } 
  return result;
}


void Controller::clusterServer()
{
  if(this->myOF.size() > 0){
        this->optic_flow = fillOpticFlowArray();

        // Preprocess the incoming data 
        double tmp_FoE_x;
        tmp_FoE_x = (double) FoE_x;

        prepEstimateClustersCPP(this->optic_flow, tmp_FoE_x, this->x_norm, this->y_norm, this->u_norm, this->v_norm, this->TTC, this->ang_norm, TTC_norm);

        // Cluster the incoming flow
        auto dbscan = DBSCAN<Controller::vec2f, float>();
        std::vector<Controller::vec2f> DBSCANdata;
        
        // Get the clustering vector
        DBSCANdata = fillDBSCANdata();

        // Cluster the OF
        dbscan.Run(&DBSCANdata, 2, 0.2f, 10);
        
        // Fill vectors with noise and clusters
        auto noise = dbscan.Noise;
        std::vector<std::vector<uint>> clusters = dbscan.Clusters;

        // Export .csv with clustering id's for check
        for (int i = 0; i < clusters.size(); i++) {
          for (int j = 0; j < clusters[i].size(); j++) {
            int tmp_idx;
            tmp_idx = clusters[i][j];
            this->Cluster_rec_file << this->last_ts << ",";
            this->Cluster_rec_file << this->x_norm[tmp_idx] << ",";
            this->Cluster_rec_file << this->y_norm[tmp_idx] << ",";
            this->Cluster_rec_file << i+1 << ",";
            this->Cluster_rec_file << this->TTC_norm[tmp_idx] << std::endl;
          }
        } 
   
        //Post process info to get cluster bounding boxes
        if (clusters.size() > 0) {
          this->idx = fillIdx(clusters);

          postEstimateClustersCPP(this->x_norm, this->y_norm, this->TTC, this->idx, this->boxes, this->TTCs);

          // Get avoidance command
          avoidanceCommand(this->boxes, this->TTCs, tmp_FoE_x, &col_timer, &rolling,
                   &roll_command);

          // Publish roll command (with a max rate of 'waittime')
          int32_t pub_roll;
          double waittime = 3.0;
          // Wait 'waittime' seconds before new roll
          if ((ros::Time::now().toSec() - prev_roll) >= waittime) { 
            pub_roll = (int32_t) this->roll_command;
            if (pub_roll != 0){
              prev_roll = ros::Time::now().toSec();
            }
          } else {
            pub_roll = (int32_t) 0;
          }

          this->roll_msg.data = pub_roll;

          // Publish roll rommand (-1, 0 or 1)          
          if (this->roll_pub.getNumSubscribers() > 0){
            this->roll_pub.publish(this->roll_msg);
          }

        } else {
          ROS_INFO("No clusters with core points detected.");
        }
    }
}

void Controller::control_job() {

    // Set rate for clustering
    ros::Rate rate_1(5);
    int cnt = 0;

    while(1) {
      this->clusterServer();
      rate_1.sleep();
    }
}

// Constructor
Controller::Controller() {
    try {
        control_job_ = std::thread(&Controller::control_job, this);
        printf("[ctrl] thread spawned!\n");
    } catch (...) {
        printf("[ctrl] can't start thread!\n");
    }
}

// Destructor
Controller::~Controller() {
    // fflush all files
        control_job_.detach();
    printf("[ctrl] thread killed!\n");
}
