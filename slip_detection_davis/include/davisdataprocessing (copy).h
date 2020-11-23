/*
 * davisdataprocessing.h
 *
 *  Created on: Feb 17, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_
#define SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_

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
//#include "CornerDetectorFAST.h"
#include "timer.h"


namespace slip_detection_davis
{

class davis_data_processing  {
public:
	davis_data_processing(ros::NodeHandle* nodehandle ) ;
	virtual ~davis_data_processing();

//virtual bool isNoise (const dvs_msgs::Event &e)	=0;
//virtual bool isEdge (const dvs_msgs::Event &e)	=0;
//virtual bool isCorner (const dvs_msgs::Event &e) =0;
    bool isCorner (const dvs_msgs::Event &e);


protected:
  std::string detector_name_;

private:
  void DavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);

  //CornerDetector_Fast *FAST;
  //ros::NodeHandle nh_;
  ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub, test_pub; // Publish classified online events
  ros::Subscriber davis_sub_; // Subscribe data from Davis
  ros::NodeHandle pnh_;
  // statistics
  double total_time_;
  int total_events_, total_corners_;
//  ros::Publisher left_events_pub, left_packets_pub, left_event_frames_pub;
//  ros::Subscriber left_davis_sub_;
//  ros::Publisher right_events_pub, right_packets_pub, right_event_frames_pub;
//  ros::Subscriber right_davis_sub_;
//  void LeftDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);
//  void RightDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);


  // SAE
  Eigen::MatrixXd sae_[2];

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};

}
#endif /* SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_ */
