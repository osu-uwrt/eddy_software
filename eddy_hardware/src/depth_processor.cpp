#include "eddy_hardware/depth_processor.h"

int main (int argc, char** argv) {
    ros::init(argc, argv, "depth_processor");
    DepthProcessor DP;
    DP.Loop();
}

// Constructor
DepthProcessor::DepthProcessor():
 depth_sub = nh.subscribe<eddy_msgs::depth>("/depth/raw", 1, &DepthProcessor::DepthCB, this);
 depth_pub = nh.advertise<eddy_msgs::depth>("/state/depth", 1);
 DepthProcessor::LoadParam<double>("post_IIR_LPF_bandwidth", post_IIR_LPF_bandwidth);
 DepthProcessor::LoadParam<double>("sensor_rate", sensor_rate);

 // IIR LPF Variables
 double fc = post_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
 dt = 1.0/sensor_rate;
 alpha = 2*PI*dt*fc / (2*PI*dt*fc + 1); // Multiplier
}

template <typename T>
void DepthProcessor::LoadParam(string param, T &var) {
  try {
    if (!nh.getParam(param, var)) {
      throw 0;
    }
  }
  catch(int e) {
    string nhNameSpace = nh.getNamespace();
    ROS_ERROR("Depth Processor Namespace: %s", nhNameSpace.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", nhNameSpace.c_str(), param.c_str());
    ros::shutdown();
  }
}


// Apply IIR LPF
void DepthProcessor::SmoothDataIIR() {
  DepthMsg.depth = alpha*DepthMsg.depth + (1-alpha)*prev_depth;
  prev_depth = DepthMsg.depth;
}


void DepthProcessor::DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg) {
  DepthMsg.header = DepthMsg -> header;
  DepthMsg.depth = DepthMsg -> depth;
  DepthMsg.pressure = DepthMsg -> pressure;
  DepthMsg.temp = DepthMsg -> temp;
  DepthMsg.altitude = DepthMsg -> altitude;
  DepthProcessor::SmoothDataIIR();

  // Publish the up-to-date data to DepthMsg
  depth_pub.publish(DepthMsg);
}

void DepthProcessor::Loop() {
  ros::Rate rate(1000);
  while (!ros::isShuttingDown()) {
    ros::spinOnce();
    rate.sleep();
  }
}
