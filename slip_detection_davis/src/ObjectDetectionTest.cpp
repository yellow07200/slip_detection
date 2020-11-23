/*
 * ObjectDetectionTest.cpp
 *
 *  Created on: Apr 22, 2019
 *      Author: raj
 */


#include <deque>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <Eigen/Eigen>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

//#include <dvs_msgs/EventArray.h>
//#include <dvs_msgs/Event.h>
//#include <davisdataprocessing.h>
//#include "CornerDetectorARC.h"
//#include "CornerDetectorFAST.h"
//#include "CornerDetectorHARRIS.h"

//#include "corner_event_detector/detector.h"
//#include "corner_event_detector/harris_detector.h"
//#include "corner_event_detector/fast_detector.h"

#include "ObjectDetection.h"
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "object_detection");

	ros::NodeHandle nh_;
	 // slip_detection_davis::davis_data_processing* process(&nh_);
	  // load parameter
	  std::string feature_type;
	  ros::param::param<std::string>("~feature_type", feature_type, "harris");
	  slip_detection_davis::ObjectDetection detect;

	 // detect.contour_extraction();



	//slip_detection_davis::CornerDetector_HARRIS x;
//  // load parameter
//  ros::param::param<std::string>("~corner_detector", corner_detector, "FAST");
//  slip_detection_davis::davis_data_processing process;
//
//  // create feature detecotr
//  if (feature_type == "harris")
//  {
//    ROS_INFO("Using Harris detector.");
//process = new slip_detection_davis::CornerDetector_HARRIS;
//  }
//  else if (corner_detector == "FAST")
//  {
//    ROS_INFO("Using fast detector.");
//process = new slip_detection_davis::CornerDetector_Fast;
//  }
//  else if (corner_detector == "ARC")
//  {
//    ROS_INFO("Using fast detector.");
//  process = new slip_detection_davis::CornerDetector_ARC;
//  }
//  else
//    {
//      ROS_ERROR("Feature type '%s' is unknown.", feature_type.c_str());
//      return 1;
//    }

  // run
  ros::spin();


  return 0;
}
