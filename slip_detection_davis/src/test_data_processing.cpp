/*
 * davisdataprocessing.cpp
 *
 *  Created on: Feb 17, 2019
 *      Author: raj
 */

#include "davisdataprocessing.h"

namespace slip_detection_davis
{
davis_data_processing::davis_data_processing(bool detect) {
	if (detect)
	  {
	corner_events_pub =nh_.advertise<dvs_msgs::EventArray>("/corner_events", 1);
    davis_sub_ = nh_.subscribe("/dvs/events", 0, &davis_data_processing::DavisCallback, this);
    ROS_INFO("constructor Data processing");

	  }
//    noise_events_pub=nh_.advertise<dvs_msgs::Event>("/noise_events", 1);
//    edge_events_pub =nh_.advertise<dvs_msgs::Event>("/edge_events", 1);


//    davis_sub_ = nh_.subscribe("/davis_left/events", 0, &davis_data_processing::LeftDavisCallback, this);
//    davis_sub_ = nh_.subscribe("/davis_right/events", 0, &davis_data_processing::RightDavisCallback, this);
//    right_events_pub = nh_.advertise<dvs_msgs::Event>("/right_davis_events", 1);
//    left_davis_sub_ = nh_.advertise<dvs_msgs::Event>("/left_davis_events", 1);

}

davis_data_processing::~davis_data_processing() {
	// TODO Auto-generated destructor stub
}

void davis_data_processing::DavisCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{

//	  dvs_msgs::EventArray packets_noise;
//	  packets_noise.header = msg->header;
//	  packets_noise.width = msg->width;
//	  packets_noise.height = msg->height;
//
//	  dvs_msgs::EventArray packets_edge;
//	  packets_edge.header = msg->header;
//	  packets_edge.width = msg->width;
//	  packets_edge.height = msg->height;

	  dvs_msgs::EventArray packets_corner;
	  packets_corner.header = msg->header;
	  packets_corner.width = msg->width;
	  packets_corner.height = msg->height;
      ROS_INFO("callback");

	  for (const auto e : msg->events)
	  {
	   // if (isNoise(e))
	   // {
	   // 	packets_noise.events.push_back(e);

	   // }
	   // if (isEdge(e))
	   // 	    {
	   // 	packets_edge.events.push_back(e);
	   // 	    }

	    if (isCorner(e))
	    	    {
	        ROS_INFO("corner detected");

	    	packets_corner.events.push_back(e);
	    	    }
	  }
  	//noise_events_pub.publish(packets_noise);
  	//edge_events_pub.publish(packets_edge);
  	corner_events_pub.publish(packets_corner);
    ROS_INFO("data processing");


}




}
