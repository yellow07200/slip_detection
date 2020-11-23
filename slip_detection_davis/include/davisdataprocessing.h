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
#include <tuple>
#include <iostream>

//#include <ShowManyImages.h>
//using namespace cv;
//using namespace std;

namespace slip_detection_davis
{

class davis_data_processing  {
public:
	davis_data_processing( ) ;
	virtual ~davis_data_processing();

//virtual bool isNoise (const dvs_msgs::Event &e)	=0;
//virtual bool isEdge (const dvs_msgs::Event &e)	=0;
//virtual bool isCorner (const dvs_msgs::Event &e) =0;
//	bool isNoise (const dvs_msgs::Event &e)	;
//	bool isEdge (const dvs_msgs::Event &e)	;
//    virtual bool isCornerFAST (const dvs_msgs::Event &e){return true;};
//   virtual  bool isCornerARC (const dvs_msgs::Event &e){return true;};
//   virtual  std::pair<bool,bool> isCorner_Edge_HARRIS (const dvs_msgs::Event &e, double th_E, double th_C){return std::make_pair(true,true);};
   virtual  double isCorner_Edge_HARRISi (const dvs_msgs::Event &e)= 0;


protected:
  std::string detector_name_;

private:
  void DavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);

  //CornerDetector_Fast *FAST;
  //ros::NodeHandle nh_;
  ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub, test_pub; // Publish classified online events
  ros::Subscriber davis_sub_; // Subscribe data from Davis
  ros::NodeHandle pnh_;
  ros::Publisher enoise_events_pub, eedge_events_pub, ecorner_events_pub;

  // statistics
  double total_time_;
  int total_events_, total_corners_;
//  ros::Publisher left_events_pub, left_packets_pub, left_event_frames_pub;
//  ros::Subscriber left_davis_sub_;
//  ros::Publisher right_events_pub, right_packets_pub, right_event_frames_pub;
//  ros::Subscriber right_davis_sub_;
//  void LeftDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);
//  void RightDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);


};

}
#endif /* SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_ */
