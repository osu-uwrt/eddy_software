#ifndef DEPTH_PROCESSOR_H
#define DEPTH_PROCESSOR_H
#include "ros/ros.h"
#include "riptide_msgs/depth.h"
using namespace std;

class DeapthProcessor {
  private:
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Publisher depth_pub;

    // IIR LPF VARIABLES
    double post_IIR_LPF_bandwidth, sensor_rate, dt, alpha, prev_depth;
    eddy_msg::depth_msg DepthMsg;
  
  public:
    DepthProcessor();
    template <typename T>
    void LoadParam (String param, T &var);
    void SmoothDataIIR();
    void Loop();
};

#endif

    
