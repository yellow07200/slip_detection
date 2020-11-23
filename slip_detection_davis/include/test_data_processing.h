/*
 * davisdataprocessing.h
 *
 *  Created on: Feb 17, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_
#define SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_

#pragma once



#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>

namespace slip_detection_davis
{

class davis_data_processing {
public:
	davis_data_processing(bool connect = true);
	virtual ~davis_data_processing();

//virtual bool isNoise (const dvs_msgs::Event &e)	=0;
//virtual bool isEdge (const dvs_msgs::Event &e)	=0;
virtual bool isCorner (const dvs_msgs::Event &e) =0;


protected:
  std::string detector_name_;

private:
  ros::NodeHandle nh_;
  ros::Publisher noise_events_pub, edge_events_pub, corner_events_pub, event_frames_pub; // Publish classified online events
  ros::Subscriber davis_sub_; // Subscribe data from Davis
  void DavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);



//  ros::Publisher left_events_pub, left_packets_pub, left_event_frames_pub;
//  ros::Subscriber left_davis_sub_;
//  ros::Publisher right_events_pub, right_packets_pub, right_event_frames_pub;
//  ros::Subscriber right_davis_sub_;
//  void LeftDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);
//  void RightDavisCallback(const dvs_msgs::EventArray::ConstPtr &msg);

};

}
#endif /* SLIP_DETECTION_DAVIS_DAVISDATAPROCESSING_H_ */
