/*
 * SoftHandDetection.cpp
 *
 *  Created on: Jun 9, 2019
 *      Author: raj
 */

#include "SoftHandDetection.h"
namespace slip_detection_davis
{



Soft_Hand_Detection::Soft_Hand_Detection():total_time_(0.), total_events_(0), total_corners_(0) {
	// TODO Auto-generated constructor stub
	object_contour_pub = pnh_.advertise<dvs_msgs::EventArray>("/slip_detection", 1);
		noise_events_sub  = pnh_.subscribe("/dvs_noise_events", 0, &Soft_Hand_Detection::NoiseEventsCallback, this);
		edge_events_sub   = pnh_.subscribe("/dvs_edge_events", 0, &Soft_Hand_Detection::EdgeEventsCallback, this);
		corner_events_sub = pnh_.subscribe("/dvs_corner_events", 0, &Soft_Hand_Detection::CornerEventsCallback, this);

		enoise_events_sub  = pnh_.subscribe("/edvs_noise_events", 0, &Soft_Hand_Detection::eNoiseEventsCallback, this);
		eedge_events_sub   = pnh_.subscribe("/edvs_edge_events", 0, &Soft_Hand_Detection::eEdgeEventsCallback, this);
		ecorner_events_sub = pnh_.subscribe("/edvs_corner_events", 0, &Soft_Hand_Detection::eCornerEventsCallback, this);


		ft_bias_sub  = pnh_.subscribe("/transformed_world", 0, &Soft_Hand_Detection::FTBiasedCallback, this);
				ft_raw_sub   = pnh_.subscribe("/netft_data", 0, &Soft_Hand_Detection::FTRawCallback, this);
//				gripper_sub = pnh_.subscribe("/robot/end_effector/left_gripper/state", 0, &Soft_Hand_Detection::CornerEventsCallback, this);


		goal_service = pnh_.advertiseService("grasp_command", &Soft_Hand_Detection::ServiceCallback, this);
		grasp_start= false;
		grasp_stop= false;
		slip_detect= false;
		max_edge_events=0;
		 raw_noise_events_=0;

}

Soft_Hand_Detection::~Soft_Hand_Detection() {
	  std::cout << "Statistics for " <<  "slip detection"  << std::endl
	  << " Total time [ns]: " << total_time_ << std::endl
	  << " Total number of events: " << total_events_ << std::endl
	  << " Total number of corners: " << total_corners_ << std::endl
	  << " Time/event [ns]: " << total_time_/(double) total_events_ << std::endl
	  << " Events/s: " << total_events_/total_time_*1e9  << std::endl
	  << " Reduction (%): " << (1.-total_corners_/(double)total_events_)*100
	  << std::endl;
}



void Soft_Hand_Detection::FTBiasedCallback(const geometry_msgs::WrenchStamped bias_ft)
{

}


void Soft_Hand_Detection::FTRawCallback(const geometry_msgs::WrenchStamped raw_ft)
{

}


//void Soft_Hand_Detection::FTRawCallback(const baxter_core_msgs::EndEffectorState gripper_left)
//{
//
//}
bool Soft_Hand_Detection::ServiceCallback(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{
	  std::cout<<"grasp_start ser" << std::endl;

	grasp_start= true;
	grasp_stop= false;

	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"grasp_stop ser" << std::endl;

	grasp_stop= true;
	grasp_start= false;

	res.status =1;


}
  return true;
}

void Soft_Hand_Detection::NoiseEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{

	noise.header = msg->header;
	noise.width = msg->width;
	noise.height = msg->height;

		for (const auto en : msg->events)
		       {
			noise.events.push_back(en);

			  }

}

void Soft_Hand_Detection::EdgeEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
	edge.header = msg->header;
	edge.width = msg->width;
	edge.height = msg->height;

			for (const auto ee : msg->events)
			       {
				edge.events.push_back(ee);

				  }
}
void Soft_Hand_Detection::CornerEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
	corner.header = msg->header;
	corner.width = msg->width;
	corner.height = msg->height;

			for (const auto ec : msg->events)
			       {
				corner.events.push_back(ec);

				  }
}


void Soft_Hand_Detection::eNoiseEventsCallback(const dvs_msgs::EventConstPtr &msgn)
{
enoise=*msgn;
//cout<<"callback raw_noise_eventsbyevent"<<enoise.ts.nsec<<endl;
//     cout<<"callback raw_noise_eventsbyevent"<<enoise.x<<endl;
//     cout<<"callbackraw_noise_eventsbyevent"<<enoise.y<<endl;
}

void Soft_Hand_Detection::eEdgeEventsCallback(const dvs_msgs::Event::ConstPtr &msge)
{
eedge=*msge;
}

void Soft_Hand_Detection::eCornerEventsCallback(const dvs_msgs::Event::ConstPtr &msgc)
{
ecorner=*msgc;
}




void Soft_Hand_Detection::SlipDetectionExperiment()
{
//
//	if (raw_start)
//	{
//	    auto start = std::chrono::system_clock::now();
//	    auto end = std::chrono::system_clock::now();
////		collect noise sample for 10 ms
//		    while((std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() != 1))
//
//		    {
//		        end = std::chrono::system_clock::now();
//		        cout<<"Taken time"<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<<" milliseconds"<<endl;
//		        cout<<"raw_noise_events_"<<raw_noise_events_<<endl;
//		//        raw_noise_events_=0;
//		    }
//	}
//
//			if (raw_stop)
//			{
//
//			}
//
//	if (grasp_start)
//	{
//		enoise
//		eedge
//		ecorner
//	}
//
//	if (grasp_stop)
//		{
//		Force_torque_msr;
//		gripper_distance;
//		gripper_percent:
//		gripper_displacement;
//
//max_edge_events;
//max_noise_events;
//max_corner_events;
//
//		}
//
//
//	if (slip_monitor_start)
//		{
//
//		if (edge >= 10){
//			slip_start
//		}
//
//		if (slip_start)
//		{
//			start_timeline;
//			Force_torque_msr;
//			slip_events_msr;
//		}
//		if (edge <= 10){
//			slip_end
//		}
//		if (slip_end)
//		{
//			stop_timeline;
//		}
//		}
//










//	dvs_msgs::Event E_noise =enoise;


	ros::Rate loop_rate(1000000000);
	while(ros::ok())
	{

	 raw_noise_events_ +=noise.events.size();
     cout<<"raw_noise_events_ OUTSIDE"<<raw_noise_events_<<endl;
     cout<<"raw_noise_eventsbyevent"<<enoise.ts.sec<<endl;

     cout<<"raw_noise_eventsbyevent"<<enoise.ts.nsec<<endl;
     cout<<"raw_noise_eventsbyevent"<<enoise.x<<endl;
     cout<<"raw_noise_eventsbyevent"<<enoise.y<<endl;



		ros::spinOnce();

     loop_rate.sleep();
	}
//	ros::spin();
//    while((std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() != 10))
//
//    {
//        end = std::chrono::system_clock::now();
//        cout<<"Time is"<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<<" milliseconds"<<endl;
//
//        cout<<"raw_noise_events_"<<raw_noise_events_<<endl;
////        raw_noise_events_=0;
//    }

// 	Collect the noise events in 10 ms frame, how much?

// Grasp start
//	if (grasp_start)
//	{
//		std::this_thread::sleep_for(std::chrono::milliseconds(10));
//	     start = std::chrono::steady_clock::now();
//
//		    while(ros::ok())
//		    {
//				 raw_noise_events_ +=noise.events.size();
//
//		 if(std::chrono::steady_clock::now() - start > std::chrono::milliseconds(10))
//		 {
//			 std::cout << "raw_total_events_ " <<  raw_noise_events_  << std::endl;
//			 raw_noise_events_=0;
//		            break;
//		 }
//		 }
// Collect the events (edges) in 10 ms frame
//		total_events_ += edge.events.size();
//		 edge_frame[1](eedge.x, eedge.y) = eedge.ts.toSec();
//
// Grasp Stop
//
// Get the maximum number of events	at the time of grasp






}








}








