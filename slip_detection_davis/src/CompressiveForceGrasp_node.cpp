/*
 * CompressiveForceGrasp_node.cpp
 *
 *  Created on: Aug 8, 2019
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
#include <CompressiveForceGrasp.h>
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

static void publish_frame_millisec_desireds(slip_detection_davis::CompressiveForceGrasp* data_ccollector)
{
	while (ros::ok())
		{
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->publish_frame_millisec();
		}
}

static void run_controller_desired(slip_detection_davis::CompressiveForceGrasp* data_ccollector)
{

//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    data_ccollector->run_controller_gripper();

}

static void FT_spin_desired(slip_detection_davis::CompressiveForceGrasp* ft)
{
	while (ros::ok())
		{
//    std::this_thread::sleep_for(std::chrono::milliseconds(10));

//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
    ft->FT_spin();
		}
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "compressive_forceh");
	ros::NodeHandle nh_;

	  slip_detection_davis::CompressiveForceGrasp* processCF;
  processCF = new slip_detection_davis::CornerDetector_HARRIS;
//
////sleepcp(3000);
std::thread th1(&publish_frame_millisec_desireds, processCF);
//std::thread th2(&FT_spin_desired, processCF);
std::thread th3(&run_controller_desired, processCF);

//th1.join();
// run
ros::spin();

delete processCF;

  return 0;
}



