#include <foe_estimator/foe_estimator.h>

// Fill OF array with values
static coder::array<double, 2U> fillOpticFlowArray()
{
  coder::array<double, 2U> result;

  // Set the size of the array (COLUMN MAJOR)
  int OFbuff_size = final_buffer.size();
  int buf_idx = 0;

  result.set_size(OFbuff_size, 4);
    
  prepMutex.lock();
  
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < (result.size(0)); idx0++) {
    
    result[idx0] = final_buffer[idx0].x * 1.0;
    result[idx0 + (result.size(0))] = final_buffer[idx0].y * 1.0;
    result[idx0 + 2*(result.size(0))] = final_buffer[idx0].u * 1.0;
    result[idx0 + 3*(result.size(0))] = final_buffer[idx0].v * 1.0;

    buf_idx++;
  }

  final_buffer.clear();
  prepMutex.unlock();

  return result;
}

void estimationServer()
{
    // Fill final OF buffer to be used by FOE estimation
    log_OF(&myOF);

    if(final_buffer.size() > 0){
       
       coder::array<double, 2U> optic_flow;
        
       optic_flow = fillOpticFlowArray();

        // Run FOE estimation
       estimateFoECPP(optic_flow, &FoE_x, &FoE_y);
    }
}

void opticflowCallback(const dvs_of::FlowPacketMsgArray::ConstPtr& msg)
{
    for (int i=0; i<msg->flowpacketmsgs.size(); i = i + 1) {
        FlowPacket OFvec;
        OFvec.t = (uint64_t)(msg->flowpacketmsgs[i].t);
        if (OFvec.t > 1e5) {
            OFvec.x = (int16_t)(msg->flowpacketmsgs[i].x);
            OFvec.y = (int16_t)(msg->flowpacketmsgs[i].y);
            OFvec.u = (float)(msg->flowpacketmsgs[i].u);
            OFvec.v = (float)(msg->flowpacketmsgs[i].v);
            
            myOF.push_back(OFvec);
        }
      }
    
    // Run FOE estimation at rate_ 
    if (ros::Time::now().toSec() - prev_time >= (1 / rate_)) { 

      // Fill OF buffers and run FOE estimation
      estimationServer();

      // Add 0.5 to FoE_x for truncating by compiler to integer
      double tmp_foe; 
      tmp_foe = FoE_x + 0.5;
      int32_t pub_x;
      pub_x = (int32_t)(tmp_foe); 
      FoE_msg.data = pub_x;

      // Publish the FOE onto its topic
      FoE_pub.publish(FoE_msg);

      prev_time = ros::Time::now().toSec();
    }
}

void log_OF(std::vector<FlowPacket> *myOF)
{
  // Fill final_buffer with current OF vector and clear myOF for new OF from subscription.
  FlowPacket OFvec_buf;               
  std::vector<FlowPacket>::iterator it = myOF->begin();
  
  prepMutex.lock();
  for(; it!=myOF->end(); it++){
    if ((*it).t >= last_ts - period_) {
      last_ts = (*it).t;
      OFvec_buf.x = (*it).x;
      OFvec_buf.y = (*it).y;
      OFvec_buf.u = (*it).u;
      OFvec_buf.v = (*it).v;
      OFvec_buf.t = (*it).t;

      final_buffer.push_back(OFvec_buf);           
    }
  }
  prepMutex.unlock();
  myOF->clear();
  
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "foe_estimator");
  ros::NodeHandle n;
  
  // Initialize subscribers and publishers
  ros::Subscriber sub = n.subscribe("/OpticFlow", 1, opticflowCallback);
  FoE_pub = n.advertise<std_msgs::Int32>("/FoEx", 1);

  ros::spin();

  return 0;
}

