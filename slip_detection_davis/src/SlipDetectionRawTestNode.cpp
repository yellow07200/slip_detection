/*
 * SlipDetectionRawTestNode.cpp
 *
 *  Created on: Oct 9, 2019
 *      Author: user
 */



#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <atomic>
#include <time.h>
#include <sstream>
#include <iostream>
#include <cstdio>
#include "ros/ros.h"
#include <slip_detection_davis/object_test.h>
#include <cstdlib>
#include <SlipDetectionRawTest.h>
//#include "CornerDetectorARC.h"
//#include "CornerDetectorFAST.h"
#include "CornerDetectorHARRIS.h"

void sleepcp(int milliseconds) // Cross-platform sleep function
{
    clock_t time_end;
    time_end = clock() + milliseconds * CLOCKS_PER_SEC/1000;
    while (clock() < time_end)
    {
//    	processs->publish_frame_millisec();
    }
}

static void publish_frame_millisec_desired(slip_detection_davis::SlipDetectionRawTest* data_ccollector)
{
	while (ros::ok())
		{
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Sampling

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->publish_frame_millisec();
		}
}


static void slip_monitor_millisec(slip_detection_davis::SlipDetectionRawTest* data_ccollector)
{
//	while (ros::ok())
//			{
//	    std::this_thread::sleep_for(std::chrono::milliseconds(5));

	data_ccollector->slip_detection_frames();   // slip monitor part
//			}
}

static void run_controller_desired(slip_detection_davis::SlipDetectionRawTest* data_ccollector)
{

//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->run_controller_gripper();

}


static void publish_complete_msg_full(slip_detection_davis::SlipDetectionRawTest* data_ccollector)
{
	while (ros::ok())
		{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));  // publish complete data Msg

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->publish_complete_msg();
		}
}

//static void FT_spin_desired(slip_detection_davis::SlipDetectionRawTest* ft)
//{
//	while (ros::ok())
//		{
////    std::this_thread::sleep_for(std::chrono::milliseconds(10));
//
////    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
////    ft->FT_spin();
//		}
//}





//static void slip_control_millisec(slip_detection_davis::soft_hand_slip_detection* data_ccollector)
//{
////	while (ros::ok())
////			{
////	    std::this_thread::sleep_for(std::chrono::milliseconds(5));
//
//	data_ccollector->slip_control_fuzzy();
////			}
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foodo");
	ros::NodeHandle nh_;

//	   double events_features[3];
//	   double cmd_grip_force;
//	   fuzzy2inputs_initialize();
//	double cc[3];
//		cc[0]=80; cc[1]=15;cc[2]=1;
//		  	cmd_grip_force = fuzzy2inputs(cc);
////		  	main_fuzzy2inputs();
//
//			  std::cout <<"cmd_grip_force----" <<cmd_grip_force<< std::endl;

	  slip_detection_davis::SlipDetectionRawTest* processs;
  processs = new slip_detection_davis::CornerDetector_HARRIS;
//	    processs = new slip_detection_davis::SlipDetectionRawTest;

//sleepcp(3000);
std::thread th1(&publish_frame_millisec_desired, processs);
//std::thread th2(&FT_spin_desired, processs);
std::thread th3(&run_controller_desired, processs);

//std::thread th4(&slip_monitor_millisec, processs);
//std::thread th6(&publish_complete_msg_full, processs);

//std::thread th5(&slip_control_millisec, processs);

//th1.join();
// run
ros::spin();

//delete processs;
//slip_detection_davis::SlipDetectionRawTest processs;
////  processs = new slip_detection_davis::CornerDetector_HARRIS;
////	    processs = new slip_detection_davis::SlipDetectionRawTest;
//
////sleepcp(3000);
//std::thread th11(publish_frame_millisec_desired, &processs);
////std::thread th2(&FT_spin_desired, processs);
//std::thread th31(run_controller_desired, &processs);
//
//std::thread th4(slip_monitor_millisec, &processs);
  return 0;
}

