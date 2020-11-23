* softhandslipdetection.h
 *
 *  Created on: Jun 12, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_SOFTHANDSLIPDETECTION_H_
#define SLIP_DETECTION_DAVIS_SRC_SOFTHANDSLIPDETECTION_H_

#pragma once

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
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
//#include <dvs_msgs/Event.h>
#include <deque>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <deque>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <slip_detection_davis/object_test.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include "timer.h"
#include <tuple>
#include <iostream>
#include <geometry_msgs/WrenchStamped.h>
#include <sys/time.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>

// Gripper
//#include <electric_gripper_baxter.h>
#include <slip_detection_davis/SlipDetectionDataComplete_analysis_final.h>
#include <baxter_gripper_server/electric_parallel_gripper.h>


namespace slip_detection_davis
{
class soft_hand_slip_detection
		{
public:
	  baxter_gripper_server::ElectricParallelGripper left_gripper;

	soft_hand_slip_detection();
	virtual ~soft_hand_slip_detection();

	   virtual  double isCorner_Edge_HARRISi (const dvs_msgs::Event &e)= 0;


    // DVS, F/T and Gripper callback
	  void Davis_Slip_detection_Callback(const dvs_msgs::EventArray::ConstPtr &msg);
      void FTBiasedCallback(const geometry_msgs::WrenchStamped::ConstPtr &bias_ft);
  	  void gripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr &gripper_left);
  	  void publish_frame_millisec();
  	  void FT_spin();
  	  void run_controller_gripper();

  	// ROS service
  	  bool ServiceCallback(object_test::Request  &req,object_test::Response &res);
  	  bool ServiceCallbacksample(object_test::Request  &req,object_test::Response &res);
//	  ElectricParallelGripper left_gripper;
//	  left_gripper("baxter_left_gripper_action/gripper_action","left", false);
//	  left_gripper.
  	  baxter_core_msgs::EndEffectorCommand command_gripper;

private:

  	  // Tuning parameters
  	uint64_t  timeSinceEpochMillisec();
	typedef std::chrono::high_resolution_clock h_clock;
	typedef std::chrono::duration<float, std::milli> duration;
  	h_clock::time_point  start = h_clock::now();
	  utils::time::Timer<std::chrono::nanoseconds> timero;

  	h_clock::time_point  t_grasp_start = h_clock::now();
  	h_clock::time_point  t_slip_start = h_clock::now();
//	  clock_t a, b;
//	  timespec aa, bb;
  	double elaspedTimeMs=0;
  	double elaspedTimeMst;
  	double elaspedTimeGrip=0;

  		  	slip_detection_davis::SlipDetectionDataComplete_analysis_final Data;
  		  	geometry_msgs::WrenchStamped FT_Biased;
  		  	baxter_core_msgs::EndEffectorState Gripper_Percent;


//  		  double HARRIS_MIN, HARRIS_MAX;


  		  double max = INT_MIN;
  		double min = INT_MAX;
  		  int maxPos = 1;
  		  int minPos = 1;

  	// Ros service server
  		  ros::ServiceServer goal_service, sample_service;

  //Subscriber and publisher and nodehandle
  ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub, test_pub; // Publish classified online events
  ros::Subscriber davis_sub_, ftbiased_sub_, ftraw_sub_,gripper_; // Subscribe data from Davis
  ros::NodeHandle pnh_;
  ros::Publisher enoise_events_pub, eedge_events_pub, ecorner_events_pub, complete_data;
  ros::Publisher gripper_command_pub;
   // time and header
  ros::Time st;
  std_msgs::Header head_data;
    // statistics
    double total_time_;
    int total_events_, total_corners_;
    //control check and data collection

    // Raw noise sample
    bool noise_sample_start, noise_sample_stop;
    int  raw_noise_events_;
    int noise_raw_max, edge_raw_max, corner_raw_max;
    // grasp start to stop
    bool grasp_start, grasp_monitor, grasp_stop, grasp_timer;
    int  grasp_noise_frame, grasp_edge_frame, grasp_corner_frame, grasp_raw_frame_events;
    int grasp_max_edge_events, grasp_count , grasp_duration_SS;

    // slip start to stop
    bool slip_start, slip_monitor, slip_end, slip_timer;
    int  slip_noise_frame, slip_edge_frame, slip_corner_frame, slip_raw_frame_events;
    int slip_total_events_edge, slip_total_events_corner, slip_total_events_noise;
    int slip_count ,slip_count_total, slip_duration_SS;

    // Counters
    int count_events,count_noise, count_corner, count_edge, t_max_data_compare;
    int count_edge_positive, count_edge_negative;
    int count_corner_positive, count_corner_negative;
    int count_noise_positive, count_noise_negative;

    // Sensor global variables
    double f_max_data_compare;
    geometry_msgs::WrenchStamped FT_readings, f_max_data,t_max_data;
    baxter_core_msgs::EndEffectorState gripper_percentage;
    bool gripper_stopped,  gripper_check;



  //  std::string condition, object, detector;

    //defined parameters
    int  frame_ms =10;
    int  threshhold_edge=200;
  	std::string condition ="10_HF_10_MF_10_V";
  	std::string object ="Metal";
  	std::string detector ="Harris";

  	int elaspedTime_grasp ;
  	int elaspedTime_slip ;


};
}
#endif /* SLIP_DETECTION_DAVIS_SRC_SOFTHANDSLIPDETECTION_H_ */

