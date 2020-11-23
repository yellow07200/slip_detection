/*
 * davisdataprocessing.cpp
 *
 *  Created on: Feb 17, 2019
 *      Author: raj
 */

#include "davisdataprocessing.h"

namespace slip_detection_davis
{
davis_data_processing::davis_data_processing():total_time_(0.), total_events_(0), total_corners_(0)


//davis_data_processing::davis_data_processing(bool detect): total_time_(0.), total_events_(0), total_corners_(0)

{
	//FAST = new CornerDetector_Fast();
//	if(detect)
//	{
//	test_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_eventss", 1);
//	corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/corner_eventss", 1);
    davis_sub_ = pnh_.subscribe("/dvs/events", 0, &davis_data_processing::DavisCallback, this);
    noise_events_pub = pnh_.advertise<dvs_msgs::EventArray>("/dvs_noise_events", 1);
    edge_events_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_edge_events", 1);
    corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events", 1);


    enoise_events_pub = pnh_.advertise<dvs_msgs::Event>("/edvs_noise_events", 1);
     eedge_events_pub  =pnh_.advertise<dvs_msgs::Event>("/edvs_edge_events", 1);
     ecorner_events_pub =pnh_.advertise<dvs_msgs::Event>("/edvs_corner_events", 1);

    //    ROS_INFO("constructor davis_data_processing");
//	}

//    noise_events_pub=nh_.advertise<dvs_msgs::Event>("/noise_events", 1);
//    edge_events_pub =nh_.advertise<dvs_msgs::Event>("/edge_events", 1);


//    davis_sub_ = nh_.subscribe("/davis_left/events", 0, &davis_data_processing::LeftDavisCallback, this);
//    davis_sub_ = nh_.subscribe("/davis_right/events", 0, &davis_data_processing::RightDavisCallback, this);
//    right_events_pub = nh_.advertise<dvs_msgs::Event>("/right_davis_events", 1);
//    left_davis_sub_ = nh_.advertise<dvs_msgs::Event>("/left_davis_events", 1);


}

davis_data_processing::~davis_data_processing() {
	  // print overall statistics
	  std::cout << "Statistics for " <<  detector_name_  << std::endl
	  << " Total time [ns]: " << total_time_ << std::endl
	  << " Total number of events: " << total_events_ << std::endl
	  << " Total number of corners: " << total_corners_ << std::endl
	  << " Time/event [ns]: " << total_time_/(double) total_events_ << std::endl
	  << " Events/s: " << total_events_/total_time_*1e9  << std::endl
	  << " Reduction (%): " << (1.-total_corners_/(double)total_events_)*100
	  << std::endl;
}
int max = INT_MIN;
int min = INT_MAX;
int maxPos = 1;
int minPos = 1;
void davis_data_processing::DavisCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{


	dvs_msgs::Event epackets_noise;
	  dvs_msgs::Event epackets_corner;
	  dvs_msgs::Event epackets_edge;

	  dvs_msgs::EventArray packets_noise;
	  packets_noise.header = msg->header;
	  packets_noise.width = msg->width;
	  packets_noise.height = msg->height;

	  dvs_msgs::EventArray packets_corner;
	  packets_corner.header = msg->header;
	  packets_corner.width = msg->width;
	  packets_corner.height = msg->height;

	  dvs_msgs::EventArray packets_edge;
	  packets_edge.header = msg->header;
	  packets_edge.width = msg->width;
	  packets_edge.height = msg->height;
    //  ROS_INFO("callback");
	  utils::time::Timer<std::chrono::nanoseconds> timer;

      for (const auto e : msg->events)
       {
  	   // ROS_INFO("DavisCallback :isCorner_Edge_HARRIS");
    	  //FAST->isCorner(e);
    		//davis_data_processing::isCorner(e);
//    	  isNoise(e);
//    	  isEdge(e);

//    	  std::pair<bool,bool> isharris= isCorner_Edge_HARRIS (e,0.01,8);
//
//    	  if (isharris.first) // Edge
//    	      	    {
//    		  packets_event.events.push_back(e);
//    	      	    }
//    	  if (isharris.second) // corner
//    	      	    {
//    	      	  packets_corner.events.push_back(e);
//    	      	    }


//	  packets_event.events.push_back(e);
//
//	  packets_corner.events.push_back(e);

//    	  if (isCornerARC(e))
//    	    {
//    	  packets_corner.events.push_back(e);
//    	    }
//    	  packets_event.events.push_back(e);


    	  double isharris= isCorner_Edge_HARRISi (e);
//    		 std::cout <<"output" << std::endl;

    	  //	  	  if (isharris>=8) // Edge
    	  //	  	      	    {
    	  //  	      	  packets_corner.events.push_back(e);
    	  //  	      	if (isharris > max) {
    	  //  	      	    max = isharris;
    	  //  	      	  //  maxPos = count;
    	  //  	      	}
    	    	      	//int maximum = -INFINITY;
    	  //	  	      	    }
    	  	  	   if (isharris<=-0.1) // corner
    	  	  	      	    {

    	  	  		epackets_edge.x=e.x;
    	  	  	epackets_edge.y=e.y;
    	  	  epackets_edge.ts=e.ts;
    	  	epackets_edge.polarity=e.polarity;
    	  	  		    	  	eedge_events_pub.publish(epackets_edge);

    	  	  		 packets_edge.events.push_back(e);
    	  		  		if (isharris < min) {
    	  		  		    min = isharris;
    	  		  		   // minPos = count;
    	  		  		}
    	  	  	      	    }


//    	  	  	  if (isharris<=-0.1 || isharris>=8) // corner
        	  	  	  if (isharris>=8) // corner
    	  	  		  	      	    {
    	  	  		packets_corner.events.push_back(e);

    	  	  		packets_noise.events.push_back(e);
    	  	  	epackets_corner.x=e.x;
    	  	  epackets_corner.y=e.y;
    	  	epackets_corner.ts=e.ts;
    	  	epackets_corner.polarity=e.polarity;
    	  	      ecorner_events_pub.publish(epackets_corner);
    	  	  		  	      	    }


    	  	  	  if (-0.1>isharris<8) // Noise
    	  	  		  	      	    {
    	  	  		packets_noise.events.push_back(e);
      	  	  		epackets_noise.x=e.x;
      	  	  		epackets_noise.y=e.y;
      	  	  	    epackets_noise.ts=e.ts;
      	  	        epackets_noise.polarity=e.polarity;
    	  	      enoise_events_pub.publish(epackets_noise);

    	  	  		  	      	    }



         }

      const auto elapsed_time_nsecs = timer.toc();

      // global stats
      total_time_ += elapsed_time_nsecs;
      total_events_ += msg->events.size();
      total_corners_ += packets_corner.events.size();

//	  for (const auto& e : msg->events)
//	  {
//	   // if (isNoise(e))
//	   // {
//	   // 	packets_noise.events.push_back(e);
//
//	   // }
//	   // if (isEdge(e))
//	   // 	    {
//	   // 	packets_edge.events.push_back(e);
//	   // 	    }
//
//	    if (isCorner(e))
//	    	    {
//	        ROS_INFO("corner detected");
//
//	    	packets_corner.events.push_back(e);
//	    	    }
//	  }


      noise_events_pub.publish(packets_noise);
      edge_events_pub.publish(packets_edge);
      corner_events_pub.publish(packets_corner);

   // ROS_INFO("DavisCallback ..........");
    // stats
    const int num_events = msg->events.size();
    if (num_events > 0)
    {
      const int num_features = packets_corner.events.size();
      const float reduction_rate = 100.*(1.-num_features/(float) num_events);
      const float reduction_factor = num_events/(float) num_features;
      const float events_per_second = float(num_events)/(elapsed_time_nsecs/1e9);
      const float ns_per_event = elapsed_time_nsecs/float(num_events);
      ROS_INFO("%s reduction rate: %.3f%% (%.0fx). Speed: %.0f e/s / %.0f ns/e.",
        detector_name_.c_str(), reduction_rate, reduction_factor,
        events_per_second, ns_per_event);
    }
    else
    {
      ROS_INFO("%s reduction rate: No events.", detector_name_.c_str());
    }

}




//bool davis_data_processing::isNoise(const dvs_msgs::Event &ex)
//{
//
//return true;
//}
//
//
//bool davis_data_processing::isEdge(const dvs_msgs::Event &ex)
//{
//
//
//
//
//
//
//
//
//	return true;
//
//}




























}
