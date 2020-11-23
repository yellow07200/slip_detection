/*
 * visual_servoing_node.cpp
 *
 *  Created on: Jan 27, 2020
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
#include <VisualServoing.h>
//#include "CornerDetectorARC.h"
//#include "CornerDetectorFAST.h"
#include "CornerDetectorHARRIS.h"




static void Corner_tracking(slip_detection_davis::Visual_Servoing* data_ccollector)
{

//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->function1();

}

static void robot_EE_translation(slip_detection_davis::Visual_Servoing* data_ccollector)
{

//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->function2();

}


static void robot_EE_orientation(slip_detection_davis::Visual_Servoing* data_ccollector)
{
	while (ros::ok())
			{
//	    std::this_thread::sleep_for(std::chrono::microseconds(500));

	data_ccollector->function3();
			}
}

static void publish_whole_data(slip_detection_davis::Visual_Servoing* data_ccollector)
{
	while (ros::ok())
		{
    std::this_thread::sleep_for(std::chrono::microseconds(500)); // Sampling

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->publish_data();
		}
}







int main(int argc, char **argv)
{
  ros::init(argc, argv, "servoing");
	ros::NodeHandle nh_;


 slip_detection_davis::Visual_Servoing* processs;
  processs = new slip_detection_davis::CornerDetector_HARRIS;
//  processs = new slip_detection_davis::CornerDetector_Fast;
//  processs = new slip_detection_davis::CornerDetector_ARC;


// Threads

//std::thread th1(&publish_frame_millisec_desired, processs);
std::thread th1(&Corner_tracking, processs);
std::thread th2(&robot_EE_translation, processs); // only when controller used
std::thread th3(&robot_EE_orientation, processs); // only when controller used
std::thread th4(&publish_whole_data, processs); // only when controller used



ros::spin();

delete processs;

  return 0;
}

