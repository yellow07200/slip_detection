/*
 * data_processing_test_node.cpp
 *
 *  Created on: Mar 26, 2019
 *      Author: raj
 */

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <davisdataprocessing.h>
#include <CornerDetector.h>

//#include "corner_event_detector/detector.h"
//#include "corner_event_detector/harris_detector.h"
//#include "corner_event_detector/fast_detector.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "slip_detection_davis");



  // load parameter
  std::string corner_detector;
  ros::param::param<std::string>("~corner_detector", corner_detector, "fast");
  slip_detection_davis::davis_data_processing* process;

  // create feature detecotr
  if (corner_detector == "harris")
  {
    ROS_INFO("Using Harris detector.");
    //process = new slip_detection_davis::CornerDetector;
  }
  else if (corner_detector == "fast")
  {
    ROS_INFO("Using fast detector.");
    process = new slip_detection_davis::CornerDetector;
  }
  else
  {
    ROS_ERROR("Feature type '%s' is unknown.", corner_detector.c_str());
    return 1;
  }
  ROS_INFO("Spinnnned once");

  // run
  ros::spin();

  delete process;

  return 0;
}



