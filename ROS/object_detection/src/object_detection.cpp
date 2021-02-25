#include <object_detection/object_detection.h>


void opticflowCallback(const dvs_of::FlowPacketMsgArray::ConstPtr& msg)
{
    // Store OF in a myOF vector for clustering
    controller->prepMutex.lock();
    for (int i=0; i<msg->flowpacketmsgs.size(); i = i + 1) {
        Controller::FlowPacket OFvec;
        OFvec.t = (uint64_t)(msg->flowpacketmsgs[i].t);
        if (OFvec.t > 1e5) {
            OFvec.x = (int16_t)(msg->flowpacketmsgs[i].x);
            OFvec.y = (int16_t)(msg->flowpacketmsgs[i].y);
            OFvec.u = (float)(msg->flowpacketmsgs[i].u);
            OFvec.v = (float)(msg->flowpacketmsgs[i].v);
            
            controller->myOF.push_back(OFvec);
        }
      }
    controller->prepMutex.unlock();

}

void foeCallback(const std_msgs::Int32::ConstPtr& msg)
{
  // Store FoE x coordinate in variable
  controller->FoE_x = msg->data;
}


Controller *controller;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "object_detection");
  ros::NodeHandle n;

  // Initialize subscribers
  ros::Subscriber of_sub = n.subscribe("/OpticFlow", 1, opticflowCallback);
  ros::Subscriber foe_sub = n.subscribe("/FoEx", 1, foeCallback);

  // Run OF clustering and object detection in seperate thread
  controller = new Controller(); 

  // Initialize the roll colland publisher
  controller->roll_pub = n.advertise<std_msgs::Int32>("/roll_command", 1);

  
  // Open a cluster log file 
  std::string myDate = currentDateTime();
  std::string ID1("/mnt/AddedSpace/Cluster_recording_");
  std::string filename1 = ID1 + myDate + ".txt";
  controller->Cluster_rec_file.open(filename1);

  if (controller->Cluster_rec_file.is_open()) {
    ROS_INFO("Opened the cluster rec file!");
  }

  if (controller->roll_pub) {
    ROS_INFO("Setup the roll-command publisher!");
  }

  ros::spin();

  controller->Cluster_rec_file.close();
  return 0;
}

