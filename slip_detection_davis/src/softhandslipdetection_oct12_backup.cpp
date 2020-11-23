* softhandslipdetection.cpp
 *
 *  Created on: oct 12, 2019
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
    complete_data =pnh_.advertise<slip_detection_davis::SlipDetectionDataComplete_analysis_final3>("/slip_complete_data", 1);
    experiment_number_pub = pnh_.advertise<std_msgs::Int32>("/experiment_number", 1);
    slip_mode_start_pub = pnh_.advertise<std_msgs::Bool>("/slip_mode_start", 1);

//ROS service
	goal_service = pnh_.advertiseService("grasp_command", &soft_hand_slip_detection::ServiceCallback, this);
	sample_service = pnh_.advertiseService("sample_command", &soft_hand_slip_detection::ServiceCallbacksample, this);
	exp_reset_service = pnh_.advertiseService("exp_reset_service", &soft_hand_slip_detection::ServiceCallbackexperiment, this);
//	only_slip_pub=pnh_.advertise<slip_detection_davis::SlipDetectionDataComplete_analysis_final3>("/only_slip_data", 1);
	// Parameters initialization




	      frame_ms=0;  grasp_duration_SS=0; slip_duration_SS=0;  elaspedTimeMst=0;
	      elaspedTime_grasp =0;
	      slip_total_events_edge =0; slip_total_events_corner =0; slip_total_events_flat =0;




// Counters
 	       count_raw=0; count_flat=0; count_corner=0; count_edge=0; grasp_count=0;
 	       count_raw_positive=0; count_raw_negative=0;
 	       count_flat_positive =0; count_flat_negative =0;
 	       count_edge_positive=0; count_edge_negative=0;
 	       count_corner_positive =0; count_corner_negative =0;

	  	// Parameters initialization
	  	// Sampling
		  noise_sample_start=false; noise_sample_stop=false;
	      raw_max=0; flat_raw_max =0; edge_raw_max =0; corner_raw_max =0;
	      raw_max_N=0; flat_raw_max_N=0; edge_raw_max_N=0; corner_raw_max_N=0;
	      raw_max_P=0; flat_raw_max_P=0; edge_raw_max_P=0; corner_raw_max_P=0;




	  	 grasp_open=false;    grasp_start=false; grasp_stop=false; grasp_monitor=false; grasp_timer=false;
	  		 grasp_max_edge_events=0;grasp_flat_frame=0; grasp_edge_frame=0; grasp_corner_frame=0;
	  		 grasp_raw_frame_events=0;grasp_duration_SS=0;   elaspedTimeMst=0; elaspedTime_grasp =0;
	  		 grasp_count_raw_positive=0; grasp_count_raw_negative=0;
	  		 grasp_count_flat_positive =0; grasp_count_flat_negative =0;
	  		 grasp_count_edge_positive=0; grasp_count_edge_negative=0;
	  		 grasp_count_corner_positive =0; grasp_count_corner_negative =0;

	  		slip_timer=false;slip_monitor=false; slip_start=false; slip_end=false; gripper_stopped=false;   gripper_check=true;
	  		 slip_flat_frame=0; slip_edge_frame=0; slip_corner_frame=0; slip_count =0; slip_raw_frame_events=0;
	  	  	 slip_count_raw_positive=0; slip_count_raw_negative=0;
	  	  	slip_count_flat_positive =0; slip_count_flat_negative =0;
	  	  	slip_count_edge_positive=0; slip_count_edge_negative=0;
	  	  	slip_count_corner_positive =0; slip_count_corner_negative =0;
	  	  	elaspedTime_slip =0; slip_count_total=0;

	  		      f_max_data_compare =0;  t_max_data_compare =0;
	  		    experiment_count=0; gripper_action=true;slip_action=true; slip_start_control=false;  gripper_action_open=true;gripper_action_close=true;
//	      command_gripper.id = 65538;
//	          command_gripper.sender = "slip_detection_node";

  events_features[0]=0;  events_features[1]=0; events_features[2]=0;
  only_slip_flat=0; only_slip_edge=0; only_slip_corner=0; cmd_grip_force=0; control_start=false; slip_seperate_monitor=false;
threshold_edge=0;
threshold_corner= 0;threshold_flat= 0;

sample_count_raw= 0; sample_count_flat= 0; sample_count_corner= 0; sample_count_edge= 0; only_slip_raw= 0;
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

    	  count_raw =count_raw +1;
	  		if(e.polarity==0)
	  		{
	  			count_raw_positive = count_raw_positive + 1;
	  		}
	  		if(e.polarity==1)
	  		{
	  			count_raw_negative = count_raw_negative + 1;
	  		}
         	    //	  std::cout <<"raw----" <<count_events<< std::endl;
//Separating Noise,edge,corner from Harris7 detector
	double th1= -0.1;//-0.0000001, 
	double th2= 8;//3;//7 //8

    	  	  	   if (isharris <= th1) // Edge %0.01
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


    	  	  	  if ((isharris > th1) &&  (isharris < th2)) // Noise
    	  	  		{
    	  	  		   packets_noise.events.push_back(e);
         	  	  		count_flat = count_flat+1;
//             	    	  std::cout <<"Noise----" <<count_noise<< std::endl;

          	  	  	if(e.polarity==0)
          	  	  	       	  	  	    	  	  		{
          	  	 	count_flat_positive = count_flat_positive+1;
          	  	          	  	  	       	  	  	    	  	  		}
          	  	          	  	  	       	  	  	    	  	  		if(e.polarity==1)
          	  	          	  	  	       	  	  	    	  	  		{
          	  	          	  	  	       	  	  	    	  	  	count_flat_negative = count_flat_negative+1;
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


//      	  	if (sample_count_flat > flat_raw_max) {flat_raw_max= sample_count_flat;}
//      	        	  	  	if (sample_count_edge > edge_raw_max) {edge_raw_max= sample_count_edge;}
//      	        	  	  	if (sample_count_corner > corner_raw_max) {corner_raw_max= sample_count_corner;}
//      	        	  	  	if (sample_count_raw > raw_max) {raw_max= sample_count_raw;}


      	  	  	if (count_flat > flat_raw_max) {flat_raw_max= count_flat;}
      	  	  	if (count_edge > edge_raw_max) {edge_raw_max= count_edge;}
      	  	  	if (count_corner > corner_raw_max) {corner_raw_max= count_corner;}
      	  	  	if (count_raw > raw_max) {raw_max= count_raw;}

      	  	  	if (count_flat_negative > flat_raw_max_N) {flat_raw_max_N= count_flat_negative;}
      	  	  	if (count_edge_negative > edge_raw_max_N) {edge_raw_max_N= count_edge_negative;}
      	  	  	if (count_corner_negative > corner_raw_max_N) {corner_raw_max_N= count_corner_negative;}
      	  	  	if (count_raw_negative > raw_max_N) {raw_max_N= count_raw_negative;}


      	  	  	if (count_flat_positive > flat_raw_max_P) {flat_raw_max_P= count_flat_positive;}
      	  	  	if (count_edge_positive > edge_raw_max_P) {edge_raw_max_P= count_edge_positive;}
      	  	  	if (count_corner_positive > corner_raw_max_P) {corner_raw_max_P= count_corner_positive;}
      	  	  	if (count_raw_positive > raw_max_P) {raw_max_P= count_raw_positive;}

      	  	  		}
      	  	  	if (noise_sample_stop)
      	  	  		{

      	  	  					  std::cout <<"raw sample noise----" <<raw_max<< std::endl;
      	  	  					  std::cout <<"raw sample noise P----" <<raw_max_P<< std::endl;
      	  	  					  std::cout <<"raw sample noise N----" <<raw_max_N<< std::endl;

      	  	  	       	    	  std::cout <<"Flat sample raw ----" <<flat_raw_max<< std::endl;
      	  	  	       	    	  std::cout <<"Flat sample raw P----" <<flat_raw_max_P<< std::endl;
      	  	  	       	    	  std::cout <<"Flat sample raw N----" <<flat_raw_max_N<< std::endl;

      	  	  	       	    	  std::cout <<"edge sample noise----" <<edge_raw_max<< std::endl;
      	  	  	       	    	  std::cout <<"edge sample noise P----" <<edge_raw_max_P<< std::endl;
      	  	  	       	    	  std::cout <<"edge sample noise N----" <<edge_raw_max_N<< std::endl;

      	  	  	       	    	  std::cout <<"corner sample noise----" <<corner_raw_max<< std::endl;
      	  	  	       	    	  std::cout <<"corner sample noise P----" <<corner_raw_max_P<< std::endl;
      	  	  	       	    	  std::cout <<"corner sample noise N----" <<corner_raw_max_N<< std::endl;



      	  	  	noise_sample_start=false;
  //    	  	  noise_sample_stop=false;
      	  	  		}
      	  	  noise_sample_stop=false;



 // Frame starts


	if (grasp_start && !grasp_timer)
	{
		noise_sample_start=false;
		noise_sample_stop=false;
		 slip_monitor=false;
//		 t_grasp_start = h_clock::now();
		 grasp_monitor=true;
		 grasp_timer=true;
	}

		 if (grasp_monitor)
		 	{
			 grasp_flat_frame = count_flat;
			 		grasp_edge_frame = count_edge;
			 		grasp_corner_frame = count_corner;
			 		grasp_raw_frame_events =count_raw;
//			 		grasp_count_raw_positive=count_raw_positive; grasp_count_raw_negative=count_raw_negative;
//			 		grasp_count_flat_positive =count_flat_positive; grasp_count_flat_negative =count_flat_negative;
//			 		grasp_count_edge_positive=count_edge_positive; grasp_count_edge_negative=count_edge_negative;
//			 		grasp_count_corner_positive =count_corner_positive; grasp_count_corner_negative =count_corner_negative;

	}

	if (grasp_stop)
		{
		 grasp_monitor=false;
		 grasp_start =false;
//		 elaspedTime_grasp =std::chrono::duration_cast<std::chrono::milliseconds>(h_clock::now() - t_grasp_start).count();
//		 grasp_duration_SS = elaspedTime_grasp;
		 grasp_count = grasp_count+1;
		 grasp_timer=false;
		 slip_monitor=true;
		 grasp_stop =false;
		}


	if (slip_monitor)
		{

		 	 	 	 	 	 grasp_flat_frame = count_flat;
					 		grasp_edge_frame = count_edge;
					 		grasp_corner_frame = count_corner;
					 		grasp_raw_frame_events =count_raw;
//					 		grasp_count_raw_positive=count_raw_positive; grasp_count_raw_negative=count_raw_negative;
//					 		grasp_count_flat_positive =count_flat_positive; grasp_count_flat_negative =count_flat_negative;
//					 		grasp_count_edge_positive=count_edge_positive; grasp_count_edge_negative=count_edge_negative;
//					 		grasp_count_corner_positive =count_corner_positive; grasp_count_corner_negative =count_corner_negative;


//		if ((count_flat >= (flat_raw_max+ (flat_raw_max*0.1))) && (count_edge >= (edge_raw_max+ (edge_raw_max*0.2)))  && !slip_timer)
//			if (((count_raw >= (raw_max+ (raw_max*0.2)))  && !slip_timer))
//				if (((count_raw >= (raw_max+ (raw_max*0.1)))  && !slip_timer))
					if (count_edge >= (edge_raw_max+ (edge_raw_max*0.01)))

		{

//  	  	  	if (count_corner > corner_raw_max) {corner_raw_max= count_corner;}

			threshold_flat= flat_raw_max;

			threshold_edge= edge_raw_max; //(edge_raw_max+ (edge_raw_max*0.2));
			threshold_corner= corner_raw_max;










					      		      slip_total_events_edge =0;
					      		      slip_total_events_corner=0;
					      			  slip_total_events_flat=0;
//			t_slip_start = h_clock::now();
			slip_start=true;
			slip_timer=true;
		}

		if (slip_start)
		{
			slip_seperate_monitor= true;
//			double sth1= -0.1;//-0.0000001,
//							double sth2= 8;//3;//7 //8

						    	  	  	   if (isharris <= th1) // Edge %0.01
						    	  	  	     {

						    	  	  		only_slip_edge = only_slip_edge+1;
			//			    	  	  		if(e.polarity==0)
			//			    	  	  		{
			//			    	  	  	count_edge_positive = count_edge_positive+1;
			//			    	  	  		}
			//			    	  	  		if(e.polarity==1)
			//			    	  	  		{
			//			    	  	    count_edge_negative = count_edge_negative+1;
			//			    	  	  		}
						//         	    	  std::cout <<"Edge----" <<count_edge<< std::endl;

						    	  	  	     }


						        	  	   if (isharris >= th2) // corner
						    	  	  		 {

						        	  		 only_slip_corner = only_slip_corner+1;
						//       	    	  std::cout <<"Corner----" <<count_corner<< std::endl;

			//			       	  	  	if(e.polarity==0)
			//			       	  	  	    	  	  		{
			//			       	  	  	    	  	  	count_corner_positive = count_corner_positive+1;
			//			       	  	  	    	  	  		}
			//			       	  	  	    	  	  		if(e.polarity==1)
			//			       	  	  	    	  	  		{
			//			       	  	  	    	  	  	count_corner_negative = count_corner_negative+1;
			//			       	  	  	    	  	  		}
						    	  	  		 }


						    	  	  	  if ((isharris > th1) &&  (isharris < th2)) // Noise
						    	  	  		{
						    	  	  		only_slip_flat = only_slip_flat+1;
						//             	    	  std::cout <<"Noise----" <<count_noise<< std::endl;

			//			          	  	  	if(e.polarity==0)
			//			          	  	  	       	  	  	    	  	  		{
			//			          	  	 	count_flat_positive = count_flat_positive+1;
			//			          	  	          	  	  	       	  	  	    	  	  		}
			//			          	  	          	  	  	       	  	  	    	  	  		if(e.polarity==1)
			//			          	  	          	  	  	       	  	  	    	  	  		{
			//			          	  	          	  	  	       	  	  	    	  	  	count_flat_negative = count_flat_negative+1;
			//			          	  	          	  	  	       	  	  	    	  	  		}
						          	  	  	    	  	  		 }

//slip_action=true;
					slip_start_control=false;

//					slip_flat_frame = only_slip_flat;
//								 slip_edge_frame = only_slip_edge ;
//								 slip_corner_frame = only_slip_corner ;

								 slip_flat_frame = only_slip_flat + threshold_flat;
								 								 slip_edge_frame = only_slip_edge + threshold_edge;
								 								 slip_corner_frame = only_slip_corner + threshold_corner;

//			 slip_flat_frame = count_flat;
//			 slip_edge_frame = count_edge;
//			 slip_corner_frame = count_corner;
//			 slip_raw_frame_events =count_raw;
//			 slip_count_raw_positive=count_raw_positive; slip_count_raw_negative=count_raw_negative;
//			 slip_count_flat_positive =count_flat_positive; slip_count_flat_negative =count_flat_negative;
//			 slip_count_edge_positive=count_edge_positive; slip_count_edge_negative=count_edge_negative;
//			 slip_count_corner_positive =count_corner_positive; slip_count_corner_negative =count_corner_negative;

//						slip_total_events_edge +=count_edge;
//						slip_total_events_corner +=count_corner;
//						slip_total_events_flat +=count_flat;

				if ((count_edge < (edge_raw_max+ (edge_raw_max*0.01))))
				{
					slip_start_control=true;
					slip_end =true;
//					slip_start= false;

				}
		}

		if (slip_end)
		{

//    	  	elaspedTime_slip =std::chrono::duration_cast<std::chrono::milliseconds>(h_clock::now() - t_slip_start).count();
//    	  	  	slip_duration_SS = elaspedTime_slip;

		slip_start= false;
		slip_count = slip_count+1;
		slip_timer=false;

//		  only_slip_flat=0; only_slip_edge=0; only_slip_corner=0;

		slip_end =false;
		}

		}


}



      noise_events_pub.publish(packets_noise);
      edge_events_pub.publish(packets_edge);
      corner_events_pub.publish(packets_corner);
      //std_msgs::Int32 experiment_count_msg;
     // experiment_count_msg.data = experiment_count;

     // experiment_number_pub.publish(experiment_count_msg);

     // std_msgs::Bool slip_monitor_msg;
     // slip_monitor_msg.data = slip_monitor;

     // slip_mode_start_pub.publish(slip_monitor_msg);

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
if (grasp_start && gripper_action_close)
	{


//	left_gripper.calibrate();
	    ros::Duration(4.0).sleep();

	    left_gripper.closeGripper();

//	    ros::Duration(4.0).sleep();
//
//	    left_gripper.openGripper();
//	    ros::Duration(4.0).sleep();

		 grasp_count = grasp_count+1;
			  std::cout <<"grasp_count----" <<grasp_count<< std::endl;
			  gripper_action_close=false;

	}

if (grasp_open && gripper_action_open)
	{


//	left_gripper.calibrate();
	    ros::Duration(4.0).sleep();

	    left_gripper.openGripper();
//	    ros::Duration(4.0).sleep();
//
//	    left_gripper.openGripper();
//	    ros::Duration(4.0).sleep();

//		 grasp_count = grasp_count+1;
//			  std::cout <<"grasp_count----" <<grasp_count<< std::endl;
			  gripper_action_open=false;

	}
if (slip_start_control && slip_action)
	{
//			  std::cout <<"Inside slip_start_control----" <<grasp_count<< std::endl;
//		//events_features[0]=slip_edge_frame;
//		//events_features[1]=slip_corner_frame;
//		//events_features[2]=slip_flat_frame;
//
//		events_features[0]=80;
//		events_features[1]=15;
//		events_features[2]=1;
//		std::cout <<"Edge Input----" <<events_features[0]<< std::endl;
//		std::cout <<"Corner Input----" <<events_features[1]<< std::endl;
//		std::cout <<"flat Input----" <<events_features[2]<< std::endl;
//	  	fuzzy2inputs_initialize();
//	  	cmd_grip_force = fuzzy2inputs(events_features);
//	  	left_gripper.closeGripper_slipcontol_command(cmd_grip_force);
//
//
//		std::cout <<"cmd_grip_force----" <<cmd_grip_force<< std::endl;
//left_gripper.closeGripper_slipcontol_100();
//slip_start_control=false;
//slip_action=false;

	}

					}
}
bool soft_hand_slip_detection::ServiceCallbacksample(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{	  std::cout<<"Sampling_start Service 1" << std::endl;


	  	  			 slip_monitor=false;
	  	  			grasp_monitor=false;
	  noise_sample_start= true;
	  noise_sample_stop= false;

	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"Sampling_stop Service 2" << std::endl;

	  noise_sample_stop= true;
	  noise_sample_start= false;

	res.status =1;


}
  return true;
}

bool soft_hand_slip_detection::ServiceCallback(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{
	  std::cout<<"Grasp_start Service 3" << std::endl;

	grasp_start= true;
	grasp_stop= false;

	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"Grasp_stop Service 4" << std::endl;

	grasp_stop= true;
	grasp_start= false;

	res.status =1;


}
  return true;
}




bool soft_hand_slip_detection::ServiceCallbackexperiment(object_test::Request  &req,object_test::Response &res)
{

	if (req.event_capture_command ==0)
	{	  std::cout<<"Experiment Reset Service 5" << std::endl;


		  	  			 slip_monitor=false;
		  	  			grasp_monitor=false;


		res.status =1;

	}
if (req.event_capture_command ==1)
{
	  std::cout<<"Experiment Reset service 6" << std::endl;

      frame_ms=0;  grasp_duration_SS=0; slip_duration_SS=0;  elaspedTimeMst=0;
      elaspedTime_grasp =0;
      slip_total_events_edge =0; slip_total_events_corner =0; slip_total_events_flat =0;




// Counters
	       count_raw=0; count_flat=0; count_corner=0; count_edge=0; grasp_count=0;
	       count_raw_positive=0; count_raw_negative=0;
	       count_flat_positive =0; count_flat_negative =0;
	       count_edge_positive=0; count_edge_negative=0;
	       count_corner_positive =0; count_corner_negative =0;

  	// Parameters initialization
  	// Sampling
	  noise_sample_start=false; noise_sample_stop=false;
      raw_max=0; flat_raw_max =0; edge_raw_max =0; corner_raw_max =0;
      raw_max_N=0; flat_raw_max_N=0; edge_raw_max_N=0; corner_raw_max_N=0;
      raw_max_P=0; flat_raw_max_P=0; edge_raw_max_P=0; corner_raw_max_P=0;




  	     grasp_start=false; grasp_stop=false; grasp_monitor=false; grasp_timer=false;
  		 grasp_max_edge_events=0;grasp_flat_frame=0; grasp_edge_frame=0; grasp_corner_frame=0;
  		 grasp_raw_frame_events=0;grasp_duration_SS=0;   elaspedTimeMst=0; elaspedTime_grasp =0;
  		 grasp_count_raw_positive=0; grasp_count_raw_negative=0;
  		 grasp_count_flat_positive =0; grasp_count_flat_negative =0;
  		 grasp_count_edge_positive=0; grasp_count_edge_negative=0;
  		 grasp_count_corner_positive =0; grasp_count_corner_negative =0;

  		slip_timer=false;slip_monitor=false; slip_start=false; slip_end=false; gripper_stopped=false;   gripper_check=true;
  		 slip_flat_frame=0; slip_edge_frame=0; slip_corner_frame=0; slip_count =0; slip_raw_frame_events=0;
  	  	 slip_count_raw_positive=0; slip_count_raw_negative=0;
  	  	slip_count_flat_positive =0; slip_count_flat_negative =0;
  	  	slip_count_edge_positive=0; slip_count_edge_negative=0;
  	  	slip_count_corner_positive =0; slip_count_corner_negative =0;
  	  	elaspedTime_slip =0; slip_count_total=0;

  		      f_max_data_compare =0;  t_max_data_compare =0;
	  	  			 slip_monitor=false;
	  	  			grasp_monitor=false;

	  experiment_count = experiment_count+1;
grasp_open=true;
	  gripper_action_open=true;

	  gripper_action_close=true;
//slip_start_control =false;
	  	  //  left_gripper.openGripper();
	  	   // ros::Duration(4.0).sleep();

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





void soft_hand_slip_detection::slip_detection_frames(){
	ros::Rate rate(200);

	while (ros::ok())
		{

		if (slip_seperate_monitor)
		{
//    std::this_thread::sleep_for(std::chrono::milliseconds(5));
		    rate.sleep();
			  std::cout <<"Slip detection-------------------------------------------Count : " <<slip_count<< std::endl;
//		events_features[0]=only_slip_edge;
//		events_features[1]=only_slip_corner;
//		events_features[2]=only_slip_flat;

		events_features[0]=slip_edge_frame;
				events_features[1]=slip_corner_frame;
				events_features[2]=slip_flat_frame;
//	    control_start=true;

		std::cout <<"Edge ----" <<events_features[0]<< std::endl;
		std::cout <<"Corner ----" <<events_features[1]<< std::endl;
		std::cout <<"flat ----" <<events_features[2]<< std::endl;
//	  	fuzzy2inputs_initialize();
//	  	cmd_grip_force = fuzzy2inputs(events_features);
//	  	left_gripper.closeGripper_slipcontol_command(cmd_grip_force);
//
//
//		std::cout <<"cmd_grip_force----" <<cmd_grip_force<< std::endl;

		  only_slip_flat=0; only_slip_edge=0; only_slip_corner=0;

		  slip_seperate_monitor=false;
//		  cmd_grip_force=0;
		}


//    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
		}
}



//void soft_hand_slip_detection::slip_control_fuzzy(){
//	ros::Rate rate(50);
//
//	while (ros::ok())
//		{
//
//		if (control_start)
//		{
////    std::this_thread::sleep_for(std::chrono::milliseconds(5));
//		    rate.sleep();
//			  std::cout <<"Inside slip_start_control----" <<grasp_count<< std::endl;
////		events_features[0]=only_slip_edge;
////		events_features[1]=only_slip_corner;
////		events_features[2]=only_slip_flat;
//
//		std::cout <<"Edge Input control----" <<events_features[0]<< std::endl;
//		std::cout <<"Corner Input control----" <<events_features[1]<< std::endl;
//		std::cout <<"flat Input control----" <<events_features[2]<< std::endl;
//	  	fuzzy2inputs_initialize();
//	  	cmd_grip_force = fuzzy2inputs(events_features);
//	  	left_gripper.closeGripper_slipcontol_command(cmd_grip_force);
//		std::cout <<"cmd_grip_force----" <<cmd_grip_force<< std::endl;
//
//	    control_start=false;
//
////		  only_slip_flat=0; only_slip_edge=0; only_slip_corner=0;
////		  cmd_grip_force=0;
//		}
//
//
////    std::cout << "-------------- Thread heartbeat --------------" << std::endl;
//		}
//}



void soft_hand_slip_detection::publish_frame_millisec(){
////	 collecting complete datset
//	Data.header=head_data;
//	Data.condition= condition;
//	Data.object = object;
//	Data.detector =detector;
//	Data.frame_ms =frame_ms;
//
//	Data.sample_start= noise_sample_start;
//	Data.flat_raw_max= flat_raw_max;
//	Data.flat_raw_max_P= flat_raw_max_P;
//	Data.flat_raw_max_N= flat_raw_max_N;
//	Data.edge_raw_max= edge_raw_max;
//	Data.edge_raw_max_P= edge_raw_max_P;
//	Data.edge_raw_max_N= edge_raw_max_N;
//	Data.corner_raw_max= corner_raw_max;
//	Data.corner_raw_max_P= corner_raw_max_P;
//	Data.corner_raw_max_N= corner_raw_max_N;
//	Data.raw_max= raw_max;
//	Data.raw_max_P= raw_max_P;
//	Data.raw_max_N= raw_max_N;
//	Data.sample_stop= noise_sample_stop;
//
//	Data.Gripper_start= grasp_start ;
//
//	Data.Gripper_status=Gripper_Percent.command_sender;
//	Data.Gripper_percent= Gripper_Percent.position;
//	Data.Gripper_force= Gripper_Percent.force;
//
//	Data.GR_count_raw_frame= grasp_raw_frame_events;
//	Data.GR_count_raw_positive=grasp_count_raw_positive;
//	Data.GR_count_raw_negative=grasp_count_raw_negative;
//
//	Data.GR_count_edge_frame= grasp_edge_frame;
//	Data.GR_count_edge_positive=grasp_count_edge_positive;
//	Data.GR_count_edge_negative=grasp_count_edge_negative;
//
//	Data.GR_count_corner_frame= grasp_corner_frame;
//	Data.GR_count_corner_positive=grasp_count_corner_positive;
//	Data.GR_count_corner_negative=grasp_count_corner_negative;
//
//	Data.GR_count_flat_frame= grasp_flat_frame ;
//	Data.GR_count_flat_positive=grasp_count_flat_positive;
//	Data.GR_count_flat_negative=grasp_count_flat_negative;
//	Data.Gripper_stop= grasp_stop;
//	Data.GR_duration_SS= grasp_duration_SS;
//	Data.GR_count=grasp_count;
//	//Data.grasping_info_arr.resize(5, 0);
////			Data.grasping_info_arr[0]=grasp_start;
//	Data.slip_monitor= slip_monitor;
//
//	Data.slip_start= slip_start;
//	Data.slip_count= slip_count;
//	Data.slip_count_total=slip_count;
//	Data.slip_count_raw_frame= slip_raw_frame_events;
//		Data.slip_count_raw_positive=slip_count_raw_positive;
//		Data.slip_count_raw_negative=slip_count_raw_negative;
//
//		Data.slip_count_edge_frame= slip_edge_frame;
//		Data.slip_count_edge_positive=slip_count_edge_positive;
//		Data.slip_count_edge_negative=slip_count_edge_negative;
//
//		Data.slip_count_corner_frame= slip_corner_frame;
//		Data.slip_count_corner_positive=slip_count_corner_positive;
//		Data.slip_count_corner_negative=slip_count_corner_negative;
//
//		Data.slip_count_flat_frame= slip_flat_frame ;
//		Data.slip_count_flat_positive=slip_count_flat_positive;
//		Data.slip_count_flat_negative=slip_count_flat_negative;
//	Data.slip_total_events_edge= slip_total_events_edge;
//	Data.slip_total_events_corner= slip_total_events_corner;
//	Data.slip_total_events_noise= slip_total_events_flat;
//
//	Data.slip_stop= slip_end;
//	Data.slip_duration_SS= slip_duration_SS;
//
//	Data.ft_raw_data = FT_Biased;
//	Data.f_max_data = f_max_data;
//	Data.t_max_data = t_max_data;
//	Data.gripper_percent= Gripper_Percent.position;
//	Data.Exp_count= experiment_count;
//
//	Only_slip_data.slip_monitor= slip_monitor;
//	Only_slip_data.slip_start= slip_start;
//	Only_slip_data.slip_count= slip_count;
//	Only_slip_data.slip_count_total=slip_count;
//	Only_slip_data.slip_count_raw_frame= slip_raw_frame_events;
//	Only_slip_data.slip_count_raw_positive=slip_count_raw_positive;
//	Only_slip_data.slip_count_raw_negative=slip_count_raw_negative;
//
//	Only_slip_data.slip_count_edge_frame= slip_edge_frame;
//	Only_slip_data.slip_count_edge_positive=slip_count_edge_positive;
//	Only_slip_data.slip_count_edge_negative=slip_count_edge_negative;
//
//	Only_slip_data.slip_count_corner_frame= slip_corner_frame;
//	Only_slip_data.slip_count_corner_positive=slip_count_corner_positive;
//	Only_slip_data.slip_count_corner_negative=slip_count_corner_negative;
//
//	Only_slip_data.slip_count_flat_frame= slip_flat_frame ;
//	Only_slip_data.slip_count_flat_positive=slip_count_flat_positive;
//	Only_slip_data.slip_count_flat_negative=slip_count_flat_negative;
//	Only_slip_data.slip_total_events_edge= slip_total_events_edge;
//	Only_slip_data.slip_total_events_corner= slip_total_events_corner;
//	Only_slip_data.slip_total_events_noise= slip_total_events_flat;

//	complete_data.publish(Data);

//if (slip_monitor)
//{
//
//	only_slip_pub.publish(Only_slip_data);
//}
	 count_raw=0; count_flat=0; count_corner=0; count_edge=0;
	 count_raw_positive=0; count_raw_negative=0;
    count_flat_positive=0; count_flat_negative=0;
    count_edge_positive=0; count_edge_negative=0;
    count_corner_positive=0; count_corner_negative=0;
	f_max_data_compare=0;  t_max_data_compare=0;
}






void soft_hand_slip_detection::publish_complete_msg(){
	// collecting complete datset
	Data.header=head_data;
	Data.condition= condition;
	Data.object = object;
	Data.detector =detector;
	Data.frame_ms =frame_ms;

	Data.sample_start= noise_sample_start;
	Data.flat_raw_max= flat_raw_max;
	Data.flat_raw_max_P= flat_raw_max_P;
	Data.flat_raw_max_N= flat_raw_max_N;
	Data.edge_raw_max= edge_raw_max;
	Data.edge_raw_max_P= edge_raw_max_P;
	Data.edge_raw_max_N= edge_raw_max_N;
	Data.corner_raw_max= corner_raw_max;
	Data.corner_raw_max_P= corner_raw_max_P;
	Data.corner_raw_max_N= corner_raw_max_N;
	Data.raw_max= raw_max;
	Data.raw_max_P= raw_max_P;
	Data.raw_max_N= raw_max_N;
	Data.sample_stop= noise_sample_stop;

	Data.Gripper_start= grasp_start ;

	Data.Gripper_status=Gripper_Percent.command_sender;
	Data.Gripper_percent= Gripper_Percent.position;
	Data.Gripper_force= Gripper_Percent.force;

	Data.GR_count_raw_frame= grasp_raw_frame_events;
	Data.GR_count_raw_positive=grasp_count_raw_positive;
	Data.GR_count_raw_negative=grasp_count_raw_negative;

	Data.GR_count_edge_frame= grasp_edge_frame;
	Data.GR_count_edge_positive=grasp_count_edge_positive;
	Data.GR_count_edge_negative=grasp_count_edge_negative;

	Data.GR_count_corner_frame= grasp_corner_frame;
	Data.GR_count_corner_positive=grasp_count_corner_positive;
	Data.GR_count_corner_negative=grasp_count_corner_negative;

	Data.GR_count_flat_frame= grasp_flat_frame ;
	Data.GR_count_flat_positive=grasp_count_flat_positive;
	Data.GR_count_flat_negative=grasp_count_flat_negative;
	Data.Gripper_stop= grasp_stop;
	Data.GR_duration_SS= grasp_duration_SS;
	Data.GR_count=grasp_count;
	//Data.grasping_info_arr.resize(5, 0);
//			Data.grasping_info_arr[0]=grasp_start;
	Data.slip_monitor= slip_monitor;

	Data.slip_start= slip_start;
	Data.slip_count= slip_count;
	Data.slip_count_total=slip_count;
	Data.slip_count_raw_frame= slip_raw_frame_events;
		Data.slip_count_raw_positive=slip_count_raw_positive;
		Data.slip_count_raw_negative=slip_count_raw_negative;

		Data.slip_count_edge_frame= slip_edge_frame;
		Data.slip_count_edge_positive=slip_count_edge_positive;
		Data.slip_count_edge_negative=slip_count_edge_negative;

		Data.slip_count_corner_frame= slip_corner_frame;
		Data.slip_count_corner_positive=slip_count_corner_positive;
		Data.slip_count_corner_negative=slip_count_corner_negative;

		Data.slip_count_flat_frame= slip_flat_frame ;
		Data.slip_count_flat_positive=slip_count_flat_positive;
		Data.slip_count_flat_negative=slip_count_flat_negative;
	Data.slip_total_events_edge= slip_total_events_edge;
	Data.slip_total_events_corner= slip_total_events_corner;
	Data.slip_total_events_noise= slip_total_events_flat;

	Data.slip_stop= slip_end;
	Data.slip_duration_SS= slip_duration_SS;

	Data.ft_raw_data = FT_Biased;
	Data.f_max_data = f_max_data;
	Data.t_max_data = t_max_data;
	Data.gripper_percent= Gripper_Percent.position;
	Data.Exp_count= experiment_count;
		complete_data.publish(Data);


		sample_count_raw= 0; sample_count_flat= 0; sample_count_corner= 0; sample_count_edge= 0;
////
////	Only_slip_data.slip_monitor= slip_monitor;
////	Only_slip_data.slip_start= slip_start;
////	Only_slip_data.slip_count= slip_count;
////	Only_slip_data.slip_count_total=slip_count;
////	Only_slip_data.slip_count_raw_frame= slip_raw_frame_events;
////	Only_slip_data.slip_count_raw_positive=slip_count_raw_positive;
////	Only_slip_data.slip_count_raw_negative=slip_count_raw_negative;
////
////	Only_slip_data.slip_count_edge_frame= slip_edge_frame;
////	Only_slip_data.slip_count_edge_positive=slip_count_edge_positive;
////	Only_slip_data.slip_count_edge_negative=slip_count_edge_negative;
////
////	Only_slip_data.slip_count_corner_frame= slip_corner_frame;
////	Only_slip_data.slip_count_corner_positive=slip_count_corner_positive;
////	Only_slip_data.slip_count_corner_negative=slip_count_corner_negative;
////
////	Only_slip_data.slip_count_flat_frame= slip_flat_frame ;
////	Only_slip_data.slip_count_flat_positive=slip_count_flat_positive;
////	Only_slip_data.slip_count_flat_negative=slip_count_flat_negative;
////	Only_slip_data.slip_total_events_edge= slip_total_events_edge;
////	Only_slip_data.slip_total_events_corner= slip_total_events_corner;
////	Only_slip_data.slip_total_events_noise= slip_total_events_flat;
//

//if (slip_monitor)
//{
//
//	only_slip_pub.publish(Only_slip_data);
//}
//	 count_raw=0; count_flat=0; count_corner=0; count_edge=0;
//	 count_raw_positive=0; count_raw_negative=0;
//    count_flat_positive=0; count_flat_negative=0;
//    count_edge_positive=0; count_edge_negative=0;
//    count_corner_positive=0; count_corner_negative=0;
//	f_max_data_compare=0;  t_max_data_compare=0;
}



}

