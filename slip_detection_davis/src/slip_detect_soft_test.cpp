/*
 * slip_detect_soft_test.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: raj
 */





#include "ros/ros.h"
#include <slip_detection_davis/object_test.h>
#include <cstdlib>
#include <SoftHandDetection.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "f");
	ros::NodeHandle nh_;
	 // slip_detection_davis::davis_data_processing* process(&nh_);
	  // load parameter
	  std::string feature_type;
	  ros::param::param<std::string>("~feature_type", feature_type, "harris");
	  slip_detection_davis::Soft_Hand_Detection handle;

//	  process = new slip_detection_davis::CornerDetector_HARRIS;

//	  ros::Rate r(10);
//
//	          while(ros::ok)
//	          {
	        	  handle.SlipDetectionExperiment();

//	                  r.sleep();
//	          }
  ros::spin();

  return 0;
}

