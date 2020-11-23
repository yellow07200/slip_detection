* softhandslipdetection.cpp
 *
 *  Created on: Jun 12, 2019
 *      Author: raj
 */

#include "softhandslipdetection.h"

namespace slip_detection_davis
{
//soft_hand_slip_detection::soft_hand_slip_detection(ros::NodeHandle* nodehandle):pnh_(*nodehandle)
//soft_hand_slip_detection::soft_hand_slip_detection(): left_gripper("baxter_left_gripper_action/gripper_action","left", false),

soft_hand_slip_detection::soft_hand_slip_detection(): left_gripper("baxter_left_gripper_action/gripper_action","left", false)
,total_time_(0.), total_events_(0), total_corners_(0)
{


// Subscribers
    davis_sub_ = pnh_.subscribe("/dvs/events", 0, &soft_hand_slip_detection::Davis_Slip_detection_Callback, this);
    ftbiased_sub_ = pnh_.subscribe("/transformed_world", 0, &soft_hand_slip_detection::FTBiasedCallback, this);
    gripper_ = pnh_.subscribe("/robot/end_effector/left_gripper/state", 0, &soft_hand_slip_detection::gripperCallback, this);


//Pubishers
//    gripper_command_pub = pnh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",10);
    noise_events_pub = pnh_.advertise<dvs_msgs::EventArray>("/dvs_noise_events_soft", 1);
    edge_events_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_edge_events_soft", 1);
    corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events_soft", 1);
    complete_data =pnh_.advertise<slip_detection_davis::SlipDetectionDataComplete_analysis_final>("/slip_complete_data", 1);

//ROS service
	goal_service = pnh_.advertiseService("grasp_command", &soft_hand_slip_detection::ServiceCallback, this);
	sample_service = pnh_.advertiseService("sample_command", &soft_hand_slip_detection::ServiceCallbacksample, this);

	// Parameters initialization
	     noise_sample_start=false; noise_sample_stop=false;
	     grasp_start=false;grasp_stop=false; grasp_monitor=false;
		 grasp_timer=false;slip_timer=false;
	     slip_monitor=false; slip_start=false; slip_end=false; gripper_stopped=false;   gripper_check=true;


	      raw_noise_events_=0;
	      grasp_max_edge_events=0;grasp_noise_frame=0; grasp_edge_frame=0; grasp_corner_frame=0;
	      slip_noise_frame=0; slip_edge_frame=0; slip_corner_frame=0; slip_count =0; slip_raw_frame_events=0;
	      count_noise=0; count_corner=0; count_edge=0; grasp_count=0; grasp_raw_frame_events=0; count_edge_positive=0; count_edge_negative=0;
	      frame_ms=0; count_events=0; grasp_duration_SS=0; slip_duration_SS=0;  elaspedTimeMst=0;
	      elaspedTime_grasp =0; elaspedTime_slip =0; slip_count_total=0;
	      slip_total_events_edge =0; slip_total_events_corner =0; slip_total_events_noise =0;
	      f_max_data_compare =0;  t_max_data_compare =0;
	       count_corner_positive =0; count_corner_negative =0;
	       count_noise_positive =0; count_noise_negative =0;
	      noise_raw_max =0; edge_raw_max =0; corner_raw_max =0;
//	      command_gripper.id = 65538;
//	          command_gripper.sender = "slip_detection_node";

}



void soft_hand_slip_detection::FTBiasedCallback(const geometry_msgs::WrenchStamped::ConstPtr &biased_ft_msg)
{
	FT_Biased=*biased_ft_msg;


	double F_squared_grasp = sqrt(pow(FT_Biased.wrench.force.x,2)+ pow(FT_Biased.wrench.force.y,2)+ pow(FT_Biased.wrench.force.z,2));
					double T_squared_grasp = sqrt(pow(FT_Biased.wrench.torque.x,2)+ pow(FT_Biased.wrench.torque.y,2)+ pow(FT_Biased.wrench.torque.z,2));

//					       	    	  std::cout <<"F_squared_grasp" <<F_squared_grasp<< std::endl;
//					       	    	  std::cout <<"f_max_data.wrench.force.x" <<FT_Biased.wrench.force.x<< std::endl;
//					       	    	  std::cout <<"f_max_data.wrench.force.y" <<FT_Biased.wrench.force.y<< std::endl;
//					       	    	  std::cout <<"f_max_data.wrench.force.z" <<FT_Biased.wrench.force.x<< std::endl;

					    	  		  		if (F_squared_grasp > f_max_data_compare)
					    	  		  			{

					    	  		  		f_max_data= FT_Biased;
					    	  		  	f_max_data_compare =F_squared_grasp;
//						       	    	  std::cout <<"f_max_data_compare" <<f_max_data_compare<< std::endl;
//						       	    	  std::cout <<"f_max_data.wrench.force.x" <<f_max_data.wrench.force.x<< std::endl;
//						       	    	  std::cout <<"f_max_data.wrench.force.y" <<f_max_data.wrench.force.y<< std::endl;
//						       	    	  std::cout <<"f_max_data.wrench.force.z" <<f_max_data.wrench.force.z<< std::endl;


			//							ft_max_data_compare = sqrt(pow(ft_max_data.wrench.force.x,2)+ pow(ft_max_data.wrench.force.y,2)+ pow(ft_max_data.wrench.force.z,2));
					    	  		  			}
					    	  		  		if (T_squared_grasp > t_max_data_compare)
					    	  				    	  		  			{
					    	  				    	  		  		t_max_data= FT_Biased;
					    	  				    	  		  	t_max_data_compare =T_squared_grasp;
					    	  		//							ft_max_data_compare = sqrt(pow(ft_max_data.wrench.force.x,2)+ pow(ft_max_data.wrench.force.y,2)+ pow(ft_max_data.wrench.force.z,2));
					    	  				    	  		  			}
}



void soft_hand_slip_detection::gripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr &gripper_msg)
{
	Gripper_Percent=*gripper_msg;
}

void soft_hand_slip_detection::Davis_Slip_detection_Callback(const dvs_msgs::EventArray::ConstPtr &msg)
{
// Packets definition
	  head_data= msg->header;
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


	  dvs_msgs::EventArray packets_raw;
	  packets_edge.header = msg->header;
	  packets_edge.width = msg->width;
	  packets_edge.height = msg->height;

// Analysing callback packets
      for (const auto e : msg->events)
{
    	  double isharris= isCorner_Edge_HARRISi (e);   // Harris Detector

    	  count_events =count_events +1;
         	    //	  std::cout <<"raw----" <<count_events<< std::endl;
//Separating Noise,edge,corner from Harris7 detector
	double th1=-0.1;//-0.0000001, 
	double th2=8;//3;//7 //8

    	  	  	   if (isharris <th1) // Edge %0.01
    	  	  	     {
    	  	  		   packets_edge.events.push_back(e);
    	  		  		if (isharris < min)
    	  		  		{
    	  		  		    min = isharris;
    	  		  		   // minPos = count;
    	  		  		}
    	  	  		count_edge = count_edge+1;
    	  	  		if(e.polarity==0)
    	  	  		{
    	  	  	count_edge_positive = count_edge_positive+1;
    	  	  		}
    	  	  		if(e.polarity==1)
    	  	  		{
    	  	    count_edge_negative = count_edge_negative+1;
    	  	  		}
//         	    	  std::cout <<"Edge----" <<count_edge<< std::endl;

    	  	  	     }


        	  	   if (isharris >= th2) // corner
    	  	  		 {
    	  	  		   packets_corner.events.push_back(e);
    	  	  		if (isharris > max)
    	  	  		    {
    	  	  		    	max = isharris;
    	  	  		    	// minPos = count;
    	  	  		    }
       	  	  		count_corner = count_corner+1;
//       	    	  std::cout <<"Corner----" <<count_corner<< std::endl;

       	  	  	if(e.polarity==0)
       	  	  	    	  	  		{
       	  	  	    	  	  	count_corner_positive = count_corner_positive+1;
       	  	  	    	  	  		}
       	  	  	    	  	  		if(e.polarity==1)
       	  	  	    	  	  		{
       	  	  	    	  	  	count_corner_negative = count_corner_negative+1;
       	  	  	    	  	  		}
    	  	  		 }


    	  	  	  if ((th1 > isharris) ||  (isharris < th2)) // Noise
    	  	  		{
    	  	  		   packets_noise.events.push_back(e);
          	  	  		count_noise = count_noise+1;
//             	    	  std::cout <<"Noise----" <<count_noise<< std::endl;

          	  	  	if(e.polarity==0)
          	  	  	       	  	  	    	  	  		{
          	  	  	       	  	  	    	  	  	count_noise_positive = count_noise_positive+1;
          	  	  	       	  	  	    	  	  		}
          	  	  	       	  	  	    	  	  		if(e.polarity==1)
          	  	  	       	  	  	    	  	  		{
          	  	  	       	  	  	    	  	  	count_noise_negative = count_noise_negative+1;
          	  	  	       	  	  	    	  	  		}
          	  	  	    	  	  		 }




//    	if (  	  	Gripper_Percent.position >=95 )
//    	{
//    		raw_noise_events_=count_events;
//    	}
//
//
//  if   	(Gripper_Percent.position <= 90 & gripper_check )
//  {
//	  grasp_start=true;
//		 auto t_gripper_start = std::chrono::high_resolution_clock::now();
//	  duration elapsed_gripper = h_clock::now() - t_gripper_start;
//	  elaspedTimeGrip +=  elapsed_gripper.count();
////5 sec in ms
//	  if(elaspedTimeGrip > 5000)
//	  {
//		  gripper_stopped =true;
//	  }
//  }
//
//
//  if   	(gripper_stopped )
//  {
//		 grasp_start =false;
//		 grasp_stop =true;
//  gripper_check=false;}



    	  	  	if (noise_sample_start)
    	  	  		{
//if (slip_monitor && grasp_monitor)
//{
//	count_noise=0; count_corner=0; count_edge=0;
//	count_events=0; count_edge_positive=0; count_edge_negative=0;
//    	  	  	 grasp_start =0;
//    	  	  	 grasp_count =0 ;
//    	  	  	 grasp_noise_frame  =0;
//    	  	  	 grasp_edge_frame =0;
//    	  	  	 grasp_corner_frame =0;
//    	  	  	grasp_raw_frame_events =0;
//    	  	  	 grasp_stop =0;
//    	  	  	 grasp_duration_SS =0;
//
//
//    	  	  	 slip_start =0;
//    	  	  	slip_count =0;
//    	  	  	slip_count =0;
//    	  	  	slip_noise_frame =0;
//    	  	  	 slip_edge_frame =0;
//    	  	  	 slip_corner_frame =0;
//    	  	  	 slip_total_events_edge =0;
//    	  	  	 slip_total_events_corner =0;
//    	  	  	 slip_total_events_noise =0;
//    	  	  	noise_raw_max =0; edge_raw_max =0; corner_raw_max =0;
//	  			 slip_monitor=false;
//	  			grasp_monitor=false;
//}
    	  	  	if (count_noise > noise_raw_max) {noise_raw_max= count_noise;}
    	  	  	if (count_edge > edge_raw_max) {edge_raw_max= count_edge;}
    	  	  	if (count_corner > corner_raw_max) {corner_raw_max= count_corner;}
    	  	  	if (count_events > raw_noise_events_) {raw_noise_events_= count_events;}


    	  	  		}
    	  	  	if (noise_sample_stop)
    	  	  		{


    	  	  	       	    	  std::cout <<"Noise sample noise----" <<noise_raw_max<< std::endl;
    	  	  	       	    	  std::cout <<"edge sample noise----" <<edge_raw_max<< std::endl;
    	  	  	       	    	  std::cout <<"corner sample noise----" <<corner_raw_max<< std::endl;
    	  	  	       	    	  std::cout <<"raw sample noise----" <<raw_noise_events_<< std::endl;

    	  	  	noise_sample_start=false;
    	  	  noise_sample_stop=false;
    	  	  		}



 // Frame starts


	if (grasp_start && !grasp_timer)
	{
		noise_sample_start=false;
		noise_sample_stop=false;
		 slip_monitor=false;
		 t_grasp_start = h_clock::now();
		 grasp_monitor=true;
		 grasp_timer=true;
	}

		 if (grasp_monitor)
		 	{
		grasp_noise_frame = count_noise;
		grasp_edge_frame = count_edge;
		grasp_corner_frame = count_corner;
		grasp_raw_frame_events =count_events;

	}

	if (grasp_stop)
		{
		 grasp_monitor=false;
		 grasp_start =false;
		 elaspedTime_grasp =std::chrono::duration_cast<std::chrono::milliseconds>(h_clock::now() - t_grasp_start).count();
		 grasp_duration_SS = elaspedTime_grasp;
		 grasp_count = grasp_count+1;
		 grasp_timer=false;
		 slip_monitor=true;
		 grasp_stop =false;
		}


	if (slip_monitor)
		{

		grasp_noise_frame = count_noise;
				grasp_edge_frame = count_edge;
				grasp_corner_frame = count_corner;
				grasp_raw_frame_events =count_events;

		if ((count_edge >= (edge_raw_max+ (edge_raw_max*0.1))) && !slip_timer)
		{

					      		      slip_total_events_edge =0;
					      		      slip_total_events_corner=0;
					      			  slip_total_events_noise=0;
			t_slip_start = h_clock::now();
			slip_start=true;
			slip_timer=true;
		}

		if (slip_start)
		{
			slip_noise_frame = count_noise;
			slip_edge_frame = count_edge;
			slip_corner_frame = count_corner;
			slip_raw_frame_events =count_events;
						slip_total_events_edge +=count_edge;
						slip_total_events_corner +=count_corner;
						slip_total_events_noise +=count_noise;

				if (slip_edge_frame <= edge_raw_max)
				{
					slip_end =true;
//					slip_start= false;

				}
		}

		if (slip_end)
		{

    	  	elaspedTime_slip =std::chrono::duration_cast<std::chrono::milliseconds>(h_clock::now() - t_slip_start).count();
    	  	  	slip_duration_SS = elaspedTime_slip;

		slip_start= false;
		slip_count = slip_count+1;
		slip_timer=false;
		slip_end =false;
		}

		}


}



      noise_events_pub.publish(packets_noise);
      edge_events_pub.publish(packets_edge);
      corner_events_pub.publish(packets_corner);
      packets_edge.events.clear();
      packets_corner.events.clear();
      packets_noise.events.clear();


}

soft_hand_slip_detection::~soft_hand_slip_detection()
{
	    	  std::cout <<"Harris Threshold min----" <<min<< std::endl;
	    	  std::cout <<"Harris Threshold max----" <<max<< std::endl;

}



void soft_hand_slip_detection::run_controller_gripper()
{

	while (ros::ok())
					{
if (grasp_start)
	{

	for (int i=0; i<=11; i++)
	{


	    left_gripper.closeGripper();

	    ros::Duration(4.0).sleep();

	    left_gripper.openGripper();
	    ros::Duration(4.0).sleep();

		 grasp_count = grasp_count+1;
			  std::cout <<"grasp_count----" <<grasp_count<< std::endl;

		 if (grasp_count ==10)
		 	 	 {grasp_stop =true;}
	}
	}
					}
//	while (ros::ok())
//			{
//		baxter_core_msgs::EndEffectorCommand command;
//		    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
//		    command.args = "{\"position\": 100.0}";
//		    command.id = 65538;
//			}
//}
}

bool soft_hand_slip_detection::ServiceCallback(object_test::Request  &req,object_test::Response &res)
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

bool soft_hand_slip_detection::ServiceCallbacksample(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{
	  std::cout<<"grasp_start ser" << std::endl;
	  count_noise=0; count_corner=0; count_edge=0;
	  	count_events=0; count_edge_positive=0; count_edge_negative=0;
	      	  	  	 grasp_start =0;
	      	  	  	 grasp_count =0 ;
	      	  	  	 grasp_noise_frame  =0;
	      	  	  	 grasp_edge_frame =0;
	      	  	  	 grasp_corner_frame =0;
	      	  	  	grasp_raw_frame_events =0;
	      	  	  	 grasp_stop =0;
	      	  	  	 grasp_duration_SS =0;


	      	  	  	 slip_start =0;
	      	  	  	slip_count =0;
	      	  	  	slip_count =0;
	      	  	  	slip_noise_frame =0;
	      	  	  	 slip_edge_frame =0;
	      	  	  	 slip_corner_frame =0;
	      	  	  	 slip_total_events_edge =0;
	      	  	  	 slip_total_events_corner =0;
	      	  	  	 slip_total_events_noise =0;
	      	  	  	noise_raw_max =0; edge_raw_max =0; corner_raw_max =0;
	  	  			 slip_monitor=false;
	  	  			grasp_monitor=false;
	  noise_sample_start= true;
	  noise_sample_stop= false;

	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"grasp_stop ser" << std::endl;

	  noise_sample_stop= true;
	  noise_sample_start= false;

	res.status =1;


}
  return true;
}

uint64_t soft_hand_slip_detection::timeSinceEpochMillisec() {
//	std::chrono:
//
//	    get current system time: std::chrono::system_clock::now()
//	    get time since epoch: .time_since_epoch()
//	    translate the underlying unit to milliseconds: duration_cast<milliseconds>(d)
//	    translate std::chrono::milliseconds to integer (uint64_t to avoid overflow)

  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void soft_hand_slip_detection::FT_spin(){
//
//	if ((grasp_start && !grasp_timer) || ((count_edge >= threshhold_edge) && !slip_timer))
//	{
//
//			      f_max_data.wrench.force.x=0;
//			      f_max_data.wrench.force.y=0;
//			      f_max_data.wrench.force.z=0;
//			      f_max_data.wrench.torque.x=0;
//			      f_max_data.wrench.torque.y=0;
//			      f_max_data.wrench.torque.z=0;
//
//			      t_max_data.wrench.force.x=0;
//			      t_max_data.wrench.force.y=0;
//			      t_max_data.wrench.force.z=0;
//			      t_max_data.wrench.torque.x=0;
//			      t_max_data.wrench.torque.y=0;
//			      t_max_data.wrench.torque.z=0;
//	      f_max_data_compare=0;
//  		  	t_max_data_compare =0;
//
//	}
//
//
//	if (grasp_monitor || slip_start)
//	{
//	double F_squared_grasp = sqrt(pow(FT_Biased.wrench.force.x,2)+ pow(FT_Biased.wrench.force.y,2)+ pow(FT_Biased.wrench.force.z,2));
//					double T_squared_grasp = sqrt(pow(FT_Biased.wrench.torque.x,2)+ pow(FT_Biased.wrench.torque.y,2)+ pow(FT_Biased.wrench.torque.z,2));
//
////					       	    	  std::cout <<"F_squared_grasp" <<F_squared_grasp<< std::endl;
////					       	    	  std::cout <<"f_max_data.wrench.force.x" <<FT_Biased.wrench.force.x<< std::endl;
////					       	    	  std::cout <<"f_max_data.wrench.force.y" <<FT_Biased.wrench.force.y<< std::endl;
////					       	    	  std::cout <<"f_max_data.wrench.force.z" <<FT_Biased.wrench.force.x<< std::endl;
//
//					    	  		  		if (F_squared_grasp > f_max_data_compare)
//					    	  		  			{
//
//					    	  		  		f_max_data= FT_Biased;
//					    	  		  	f_max_data_compare =F_squared_grasp;
////						       	    	  std::cout <<"f_max_data_compare" <<f_max_data_compare<< std::endl;
////						       	    	  std::cout <<"f_max_data.wrench.force.x" <<f_max_data.wrench.force.x<< std::endl;
////						       	    	  std::cout <<"f_max_data.wrench.force.y" <<f_max_data.wrench.force.y<< std::endl;
////						       	    	  std::cout <<"f_max_data.wrench.force.z" <<f_max_data.wrench.force.z<< std::endl;
//
//
//			//							ft_max_data_compare = sqrt(pow(ft_max_data.wrench.force.x,2)+ pow(ft_max_data.wrench.force.y,2)+ pow(ft_max_data.wrench.force.z,2));
//					    	  		  			}
//					    	  		  		if (T_squared_grasp > t_max_data_compare)
//					    	  				    	  		  			{
//					    	  				    	  		  		t_max_data= FT_Biased;
//					    	  				    	  		  	t_max_data_compare =T_squared_grasp;
//					    	  		//							ft_max_data_compare = sqrt(pow(ft_max_data.wrench.force.x,2)+ pow(ft_max_data.wrench.force.y,2)+ pow(ft_max_data.wrench.force.z,2));
//					    	  				    	  		  			}
//	}
}

void soft_hand_slip_detection::publish_frame_millisec(){
	// collecting complete datset
	Data.header=head_data;
	Data.condition= condition;
	Data.object = object;
	Data.detector =detector;
	Data.frame_ms =frame_ms;

	Data.sample_start= noise_sample_start;
	Data.noise_sample_events= noise_raw_max;
	Data.edge_sample_events= edge_raw_max;
	Data.corner_sample_events= corner_raw_max;
	Data.raw_sample_total_events_frame= raw_noise_events_;
	Data.sample_stop= noise_sample_stop;


	Data.grasp_status="opening";
	Data.grasp_start= grasp_start ;
	Data.grasp_count= grasp_count ;
	Data.grasp_noise_frame= grasp_noise_frame ;
	Data.grasp_edge_frame= grasp_edge_frame;
	Data.grasp_corner_frame= grasp_corner_frame;
	Data.grasp_raw_total_events_frame= grasp_raw_frame_events;
	Data.grasp_stop= grasp_stop;
	Data.grasp_duration_SS= grasp_duration_SS;


	Data.count_edge_positive=count_edge_positive;
	Data.count_edge_negative=count_edge_negative;
	Data.count_corner_positive=count_corner_positive;
	Data.count_corner_negative=count_corner_negative;
	Data.count_noise_positive=count_noise_positive;
	Data.count_noise_negative=count_noise_negative;
	//Data.grasping_info_arr.resize(5, 0);
//			Data.grasping_info_arr[0]=grasp_start;

	Data.slip_start= slip_start;
	Data.slip_count= slip_count;
	Data.slip_count_total=slip_count;
	Data.slip_noise_frame= slip_noise_frame;
	Data.slip_edge_frame= slip_edge_frame;
	Data.slip_corner_frame= slip_corner_frame;
	Data.slip_total_events_edge= slip_total_events_edge;
	Data.slip_total_events_corner= slip_total_events_corner;
	Data.slip_total_events_noise= slip_total_events_noise;

	Data.slip_stop= slip_end;
	Data.slip_duration_SS= slip_duration_SS;
	Data.ft_raw_data = FT_Biased;

	Data.f_max_data = f_max_data;
	Data.t_max_data = t_max_data;
	Data.gripper_percent= Gripper_Percent.position;

	complete_data.publish(Data);
	count_noise=0; count_corner=0; count_edge=0;
	count_events=0; count_edge_positive=0; count_edge_negative=0;
	//noise_raw_max=0; edge_raw_max=0; corner_raw_max=0; raw_noise_events_=0;
     count_edge_positive=0; count_edge_negative=0;
     count_corner_positive=0; count_corner_negative=0;
     count_noise_positive=0; count_noise_negative=0;
	f_max_data_compare=0;  t_max_data_compare=0;
}





}

