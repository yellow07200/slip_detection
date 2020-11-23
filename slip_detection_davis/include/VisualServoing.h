/*
 * VisualServoing.h
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_
#define SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_



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

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
// Gripper
//#include <electric_gripper_baxter.h>
#include <slip_detection_davis/SlipDetectionDataComplete_analysis_final3.h>

#include <stddef.h>
#include <stdlib.h>



namespace slip_detection_davis
{
class Visual_Servoing {
public:
	Visual_Servoing();
	virtual ~Visual_Servoing();


	   virtual  double isCorner_Edge_HARRISi (const dvs_msgs::Event &e)= 0;
//	   virtual  bool isCornerFAST (const dvs_msgs::Event &e)= 0;
//	   virtual  bool isCornerARC (const dvs_msgs::Event &e)= 0;



    // callback and functions
	  void Davis_feature_Callback(const dvs_msgs::EventArray::ConstPtr &msg);
  	  void function1();
  	  void function2();
  	  void function3();
  	  void publish_data();

  	  // ROS service
  	  bool ServiceCallback1(object_test::Request  &req,object_test::Response &res);
  	  bool ServiceCallback2(object_test::Request  &req,object_test::Response &res);

    	double th1= -0.001;//-0.0000001,
    	double th2= 8;//3;//7 //8
    	int e_max, c_max;
private:
        Eigen::MatrixXd sae;
  	  static const int sensor_width_ = 240;
  	  static const int sensor_height_ = 180;
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

  		  	slip_detection_davis::SlipDetectionDataComplete_analysis_final3 Data;
//  		  	slip_detection_davis::SlipDetectionDataComplete_analysis_final3 Only_slip_data;

  		  	geometry_msgs::WrenchStamped FT_Biased;

  		  dvs_msgs::Event C_e, E_e, C_c;


  	// Ros service server
  ros::ServiceServer goal_service, sample_service, exp_reset_service;

  //Subscriber and publisher and nodehandle
  ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub,complete_data, centtoid_pub; // Publish classified online events
  ros::Subscriber davis_sub_; // Subscribe data from Davis
  ros::NodeHandle pnh_;
   // time and header
  ros::Time st;
  std_msgs::Header head_data;

    // Counters
    int count_raw, count_flat, count_corner, count_edge, t_max_data_compare;




  //  std::string condition, object, detector;

    //defined parameters
    int  frame_ms =1;
    int  threshhold_edge=200;
  	std::string condition ="10_HF_10_MF_10_V";
  	std::string object ="Metal";
  	std::string detector ="Harris";

  	int elaspedTime_grasp ;
  	int elaspedTime_slip ;

  	int control_edge_raw_max, control_corner_raw_max;
};
}
#endif /* SLIP_DETECTION_DAVIS_SRC_VISUALSERVOING_H_ */
