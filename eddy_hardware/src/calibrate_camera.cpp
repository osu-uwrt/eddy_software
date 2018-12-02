#include "eddy_hardware/calibratecamera.h"
#include <fstream>
//using namespace cv;
static const int WIDTH = 960;
static const int HEIGHT = 600;
static const std::string WIN_ORIGINAL = "Original Image";
static const std::string WIN_CHECKERBOARD = "Checkerboard Image";
static const std::string WIN_UNDISTORTED = "Undistorted Image";

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibrate_camera");
  CalibrateCamera cal;
  cal.Loop();
}

CalibrateCamera::CalibrateCamera() : nh("calibrate_camera") {
    CalibrateCamera::LoadParam<int>("numBoards", numBoards); // Number of snapshots to take from video feed
    CalibrateCamera::LoadParam<int>("numCornersHor", numCornersHor);
    CalibrateCamera::LoadParam<int>("numCornersVer", numCornersVer);
    CalibrateCamera::LoadParam<double>("frame_rate", frame_rate);
    CalibrateCamera::LoadParam<string>("camera_name", camera_name);

    sub_topic = "/"+camera_name+"/image_raw";
    raw_image_sub = nh.subscribe<sensor_msgs::Image>(sub_topic, 1, &CalibrateCamera::ImageCB, this);

    numSquares = numCornersHor * numCornersVer;
    board_sz = Size(numCornersHor, numCornersVer);

    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  // Camera Matrix
  // Aspect ratio is 4:3
    cameraMatrix = Mat(3, 3, CV_32FC1);
    cameraMatrix.ptr<float>(0)[0] = 4; // Focal length along x-axis
    cameraMatrix.ptr<float>(1)[1] = 3; // Focal length along y-axis

    namedWindow(WIN_ORIGINAL, WINDOW_NORMAL);
    namedWindow(WIN_CHECKERBOARD, WINDOW_NORMAL);
    resizeWindow(WIN_ORIGINAL, WIDTH, HEIGHT);
    resizeWindow(WIN_CHECKERBOARD, WIDTH, HEIGHT);

    calculated = false;
    pauses = 0;
}

template <typename T>
void CalibrateCamera::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Calibrate Camera Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void CalibrateCamera::ImageCB(const sensor_msgs::Image::ConstPtr &msg) {
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(!calculated) {
        pauses++;
        if(pauses > frame_rate)
        pauses = 0;
  }

    if(!cv_ptr->image.empty() && (successes <= numBoards)){
        cvtColor(cv_ptr->image, gray_image, CV_BGR2GRAY);
        bool found = findChessboardCorners(cv_ptr->image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if(found){
            cornerSubPix(gray_image, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER,30,0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }
        imshow(WIN_ORIGINAL, cv_ptr-> image);
        imshow(WIN_CHECKERBOARD, gray_image);
        waitkey(1);

        if(pauses == 0){
          image_points.push_back(corners);
          object_points.push_back(obj);
          ROS_INFO("Number of successes: %i ", successes);
          successes++;
        }
    }
    //calc coeefficients time
    if(successes>numBoards && !calculated){
      ofstream output;
      ROS_INFO("Calibrating..");
      calibrateCamera(object_points, image_points, cv_ptr->image.size(), cameraMatrix, distortionCoeffs, rvecs, tvecs);
      ROS_INFO("Calibrated standby for coefficients:")
      std::cout << "cameraMatrix:" << endl << cameraMatrix << endl << endl;
      std::cout << "Distortion Coeffs:" << endl << distortionCoeffs<< endl << endl;

      ofstream.open("cfg/" + camera_name + "_cal.yaml");

      output << "cameraMatrix:" << endl;
      output << " R0: #Row 0" << endl;
      output << "   C0: " << cameraMatrix.ptr<double>(0)[0]) << endl;
      output << "   C1: " << cameraMatrix.ptr<double>(0)[1]) << endl;
      output << "   C2: " << cameraMatrix.ptr<double>(0)[2]) << endl;
      output << " R1: #Row 1" << endl;
      output << "   C0: " << cameraMatrix.ptr<double>(1)[0]) << endl;
      output << "   C1: " << cameraMatrix.ptr<double>(1)[1]) << endl;
      output << "   C2: " << cameraMatrix.ptr<double>(1)[2]) << endl;
      output << " R2: #Row 2" << endl;
      output << "   C0: " << cameraMatrix.ptr<double>(2)[0]) << endl;
      output << "   C1: " << cameraMatrix.ptr<double>(2)[1]) << endl;
      output << "   C2: " << cameraMatrix.ptr<double>(2)[2]) << endl; 
      output << "distortionCoeffs:" << endl;
      output << " C0: " << distortionCoeffs.ptr<double>(0)[0]) << endl;
      output << " C1: " << distortionCoeffs.ptr<double>(0)[1]) << endl;
      output << " C2: " << distortionCoeffs.ptr<double>(0)[2]) << endl;
      output << " C3: " << distortionCoeffs.ptr<double>(0)[3]) << endl;
      output << " C4: " << distortionCoeffs.ptr<double>(0)[4]) << endl;

      output.close();

      ROS_INFO("cameraMatrix [0][0] %.8f", cameraMatrix.ptr<double>(0)[0]);
      ROS_INFO("cameraMatrix [0][1] %.8f", cameraMatrix.ptr<double>(0)[1]);
      ROS_INFO("cameraMatrix [0][2] %.8f", cameraMatrix.ptr<double>(0)[2]);
      ROS_INFO("cameraMatrix [1][0] %.8f", cameraMatrix.ptr<double>(1)[0]);
      ROS_INFO("cameraMatrix [1][1] %.8f", cameraMatrix.ptr<double>(1)[1]);
      ROS_INFO("cameraMatrix [1][2] %.8f", cameraMatrix.ptr<double>(1)[2]);
      ROS_INFO("cameraMatrix [2][0] %.8f", cameraMatrix.ptr<double>(2)[0]);
      ROS_INFO("cameraMatrix [2][1] %.8f", cameraMatrix.ptr<double>(2)[1]);
      ROS_INFO("cameraMatrix [2][2] %.8f", cameraMatrix.ptr<double>(2)[2]);

      ROS_INFO("Distortion. coef [0][0] %.8f", distortionCoeffs.ptr<double>(0)[0]);
      ROS_INFO("Distortion. coef [0][1] %.8f", distortionCoeffs.ptr<double>(0)[1]);
      ROS_INFO("Distortion. coef [0][2] %.8f", distortionCoeffs.ptr<double>(0)[2]);
      ROS_INFO("Distortion. coef [0][3] %.8f", distortionCoeffs.ptr<double>(0)[3]);
      ROS_INFO("Distortion. coef [0][4] %.8f", distortionCoeffs.ptr<double>(0)[4]);



      calculated = true;

      destroyWindow(WIN_CHECKERBOARD);
      namedWindow(WIN_UNDISTORTED, WINDOW_NORMAL);
      resizeWindow(WIn)
    }

    if(calculated) {
    Mat imageUndistorted;
    ros::Time time_start = ros::Time::now();
    undistort(cv_ptr->image, imageUndistorted, cameraMatrix, distortionCoeffs);
    ROS_INFO("undistorted image in %f s", (ros::Time::now() - time_start).toSec());

    sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageUndistorted).toImageMsg();

    imshow(WIN_ORIGINAL, cv_ptr->image);
    imshow(WIN_UNDISTORTED, imageUndistorted);
    waitKey(1);
  }

  cv_ptr.reset();
  }
void CalibrateCamera::Loop()
{
  ros::Rate rate(frame_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
