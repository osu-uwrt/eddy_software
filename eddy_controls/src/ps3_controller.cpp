#include "eddy_controls/ps3_controller.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "ps3_controller");
    PS3Controller ps3;
    ps3.loop();
}

PS3Controller::PS3Contoller() : nh("ps3_controller"){
    joy_sub = nh.subscribe<sensor_msgs::Joy>(/joy, 1, &PS3Controller::JoyCB, this);
    attitude_updater = nh.advertise<eddy_msgs::AttitudeCommand>("/command/attitude", 1);
    ang_accel
}