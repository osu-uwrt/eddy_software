#include "eddy_hardware/undistort_camera.h"

int main(int arc, char** argv) {
    ros::init(argc,argv, "undistort_camera");
    UndistortCamera cam;
    cam.loop();
}

UndistortCamera::UndistortCamera() : nh("undistort_camera"){
    cameraMatrix = Mat(3,3,CV_64FC1);
    
}