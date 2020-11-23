/*
 * event_process_test.cpp
 *
 *  Created on: Apr 3, 2019
 *      Author: raj
 */





#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <davisdataprocessing.h>
#include "CornerDetectorARC.h"
#include "CornerDetectorFAST.h"

//#include "corner_event_detector/detector.h"
//#include "corner_event_detector/harris_detector.h"
//#include "corner_event_detector/fast_detector.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "event_process");
	ros::NodeHandle nh_;
   slip_detection_davis::CornerDetector_Fast fast;
   slip_detection_davis::CornerDetector_ARC  ARC;



  // run
  ros::spin();

  //delete process;

  return 0;
}
