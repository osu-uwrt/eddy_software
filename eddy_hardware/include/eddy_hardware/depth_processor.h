#ifndef DEPTH_PROCESSOR_H
#define DEPTH_PROCESSOR_H
#include "ros/ros.h"
#include "eddy_msgs/depth.h"
using namespace std;

class DepthProcessor {
  private:
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Publisher depth_pub;

    // IIR LPF VARIABLES
    double post_IIR_LPF_bandwidth, sensor_rate, dt, alpha, prev_depth;
    eddy_msgs::depth DepthMsg;
  
  public:
    DepthProcessor();
    template <typename T>
    void LoadParam(string param, T &var);
    void DepthCB(const eddy_msgs::depth::ConstPtr& depth_msg);
    void SmoothDataIIR();
    void Loop();
};

#endif

    
