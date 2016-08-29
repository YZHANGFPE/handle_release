/**
 * @file release_handle_node.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include "handle_release/HandleDetection.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <math.h>

using namespace std;
using namespace cv;

namespace
{
    // windows and trackbars name
    const std::string windowName = "Line Detection Demo";
    const std::string targetThresholdTrackbarName = "Line threshold";
    const std::string houghThresholdTrackbarName = "Hough threshold";
    const std::string cannyThresholdTrackbarName = "Canny threshold";
    const std::string usage = "Usage : line_detection <path_to_input_image>\n";

    // initial and max values of the parameters of interests.
    const int initialHoughThresholdValue = 90;
    const int maxHoughThresholdValue = 500;
    const int initialTargetThresholdValue = 530;
    const int maxTargetThresholdValue = 640;
    const int initialCannyThresholdValue = 20;
    const int maxCannyThresholdValue = 500;

    //declare and initialize both parameters that are subjects to change
    int targetThreshold = initialTargetThresholdValue;
    int houghThreshold = initialHoughThresholdValue;
    int cannyThreshold = initialCannyThresholdValue;

    // distance between the current handle location and target location in the image
    float dx = 0;
    float dy = 0;
    float xdirCoef = 0.0;
    float ydirCoef = 0.0;

    void lineDetection(const Mat& src_gray, const Mat& src_display, int targetThreshold, int houghThreshold, int cannyThreshold)
    {
        // will hold the results of the detection
        std::vector<Vec3f> circles;

        // clone the colour, input image for displaying purposes
        Mat display = src_display.clone();
        Mat dst;

        Point start, end;
        start.x = targetThreshold;
        start.y = 0;
        end.x = targetThreshold;
        end.y = 400;
        line(display, start, end, Scalar(0,0,255), 5);

        // Reduce the noise so we avoid false circle detection
        GaussianBlur(src_gray, src_gray, Size(3, 3), 2, 2 );

        Canny(src_gray, dst, cannyThreshold, 3 * cannyThreshold, 3); 

        vector<Vec2f> lines;
        // detect lines
        HoughLines(dst, lines, 1, CV_PI/180, houghThreshold, 0, 0 );

        // draw lines
        int minX= 640;
        Point minPt1, minPt2;
        for( size_t i = 0; i < lines.size(); i++ )
        {
          float rho = lines[i][0], theta = lines[i][1];
          if((theta>CV_PI/180*170 && theta<CV_PI/180*190) || 
              (theta>CV_PI/180*0 && theta<CV_PI/180*10)) {
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            if (pt1.x < minX) {
              minX = pt1.x;
              minPt1 = pt1;
              minPt2 = pt2;
            }
          }
        }
        line(display, minPt1, minPt2, Scalar(0,255,0), 3, CV_AA);

        // compute dy
        float dy_gripper = targetThreshold - (minPt1.x + minPt2.x) / 2;
        dx = dy_gripper * xdirCoef;
        dy = dy_gripper * ydirCoef;

        // shows the results
        imshow( windowName, display);
    }
}

bool readTFTransform(const std::string& target_frame, const std::string& source_frame,
  const ros::Time &time, const double & timeout, Eigen::Affine3d & T_Eigen) {
  tf::TransformListener tf_listener;
  tf_listener.waitForTransform(target_frame, source_frame, time, ros::Duration(timeout) );
  tf::StampedTransform transform;
  cout << "start tf" << endl;

  try {
    tf_listener.lookupTransform (target_frame, source_frame, time, transform);
  } catch (tf::LookupException &e) {
    ROS_ERROR ("%s", e.what ());
    return false;
  } catch (tf::ExtrapolationException &e) {
    ROS_ERROR ("%s", e.what ());
    return false;
  }
  std::cout << "Finish read tf" << std::endl;

  tf::transformTFToEigen (transform, T_Eigen );
  return true;
}


void calculateGripperAngle() {
  Eigen::Affine3d T_Eigen;

  if (!readTFTransform("/base", "/left_gripper", ros::Time(0), 3.0, T_Eigen) ) {
      std::cout << "tf issue" << std::endl;
      return;
  }
  
  Eigen::Matrix3d Rd = T_Eigen.matrix().block<3, 3>(0, 0);
  Eigen::Matrix3f Rf = Rd.cast<float>();
  Eigen::Vector3f ydir = Rf.col(1);
  float x = ydir(0);
  float y = ydir(1);
  xdirCoef = x / sqrt(x * x + y * y);
  ydirCoef = y / sqrt(x * x + y * y);
  ROS_INFO("ydir: [%f], [%f], [%f]", ydir(0), ydir(1), ydir(2));
}

void imageCallback(const sensor_msgs::Image& msgs_image) {


  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msgs_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat src, src_gray;  

  // Read the image
  src = cv_ptr->image;

  // Convert it to gray
  cvtColor( src, src_gray, COLOR_BGR2GRAY );


  //runs the detection, and update the display
  lineDetection(src_gray, src, targetThreshold, houghThreshold, cannyThreshold);

  // get user key
  waitKey(10);

}

bool handle_req(handle_release::HandleDetection::Request  &req,
    handle_release::HandleDetection::Response &res)
{
  if (req.controlID == 0) {
    calculateGripperAngle();
    dx = 0;
    dy = 0;
  }
  res.dx = dx;
  res.dy = dy;
  ROS_INFO("sending back response: dx [%ld], dy [%ld]", (long int)res.dx, (long int)res.dy);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_handle");
  ros::NodeHandle nh;

  targetThreshold = std::max(targetThreshold, 1);
  houghThreshold = std::max(houghThreshold, 1);
  cannyThreshold = std::max(cannyThreshold, 1);

  // create the main window, and attach the trackbars
  namedWindow( windowName, WINDOW_AUTOSIZE );
  createTrackbar(targetThresholdTrackbarName, windowName, &targetThreshold, maxTargetThresholdValue);
  createTrackbar(houghThresholdTrackbarName, windowName, &houghThreshold, maxHoughThresholdValue);
  createTrackbar(cannyThresholdTrackbarName, windowName, &cannyThreshold, maxCannyThresholdValue);

  // subscribe to the image topic
  ros::Subscriber sub = nh.subscribe("/cameras/left_hand_camera/image", 1, imageCallback);

  // create service
  ros::ServiceServer service = nh.advertiseService("microwave_handle_detection", handle_req);

  ros::spin();

  return 0;
}
