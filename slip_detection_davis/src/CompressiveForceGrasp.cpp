/*
 * CompressiveForceGrasp.cpp
 *
 *  Created on: Aug 8, 2019
 *      Author: user
 */

#include "CompressiveForceGrasp.h"



namespace slip_detection_davis
{
//soft_hand_slip_detection::soft_hand_slip_detection(ros::NodeHandle* nodehandle):pnh_(*nodehandle)
//soft_hand_slip_detection::soft_hand_slip_detection(): left_gripper("baxter_left_gripper_action/gripper_action","left", false),

CompressiveForceGrasp::CompressiveForceGrasp(): left_gripper("baxter_left_gripper_action/gripper_action","left", false)
,total_time_(0.), total_events_(0), total_corners_(0)
{


// Subscribers
    davis_sub_ = pnh_.subscribe("/dvs/events", 0, &CompressiveForceGrasp::Davis_Slip_detection_Callback, this);
    ftbiased_sub_ = pnh_.subscribe("/transformed_world", 0, &CompressiveForceGrasp::FTBiasedCallback, this);
    gripper_ = pnh_.subscribe("/robot/end_effector/left_gripper/state", 0, &CompressiveForceGrasp::gripperCallback, this);


//Pubishers
    flat_events_pub = pnh_.advertise<dvs_msgs::EventArray>("/dvs_noise_events_soft", 1);
    edge_events_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_edge_events_soft", 1);
    corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events_soft", 1);
    complete_data =pnh_.advertise<slip_detection_davis::CompressiveForceGrasp_stable_Final>("/GR_complete_data", 1);

//ROS service
	goal_service = pnh_.advertiseService("grasp_command", &CompressiveForceGrasp::ServiceCallback, this);
	sample_service = pnh_.advertiseService("sample_command", &CompressiveForceGrasp::ServiceCallbacksample, this);

	// Parameters initialization
	// Sampling
    raw_max=0; flat_raw_max =0; edge_raw_max =0; corner_raw_max =0;
    raw_max_N=0; flat_raw_max_N=0; edge_raw_max_N=0; corner_raw_max_N=0;
    raw_max_P=0; flat_raw_max_P=0; edge_raw_max_P=0; corner_raw_max_P=0;
	noise_sample_start=false; noise_sample_stop=false;

	     grasp_start=false; grasp_stop=false; grasp_monitor=false; grasp_timer=false;




	       count_raw=0; count_flat=0; count_corner=0; count_edge=0; grasp_count=0;
	       count_raw_positive=0; count_raw_negative=0;
	       count_flat_positive =0; count_flat_negative =0;
	       count_edge_positive=0; count_edge_negative=0;
	       count_corner_positive =0; count_corner_negative =0;


		      grasp_max_edge_events=0;grasp_flat_frame=0; grasp_edge_frame=0; grasp_corner_frame=0;
		       grasp_raw_frame_events=0;
		       grasp_duration_SS=0;   elaspedTimeMst=0; elaspedTime_grasp =0;
		      f_max_data_compare =0;  t_max_data_compare =0;

		      grasp_count_raw_positive=0; grasp_count_raw_negative=0;
		      		grasp_count_flat_positive =0; grasp_count_flat_negative =0;
		      		grasp_count_edge_positive=0; grasp_count_edge_negative=0;
		      		grasp_count_corner_positive =0; grasp_count_corner_negative =0;

	// Gripper command
	      gripper_command_pub = pnh_.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",10);
	      command_gripper.id = 65538;
	      command_gripper.sender = "slip_detection_node";
		   gripper_stopped=false;   gripper_check=true;

//		   sae_[0] = Eigen::MatrixXd::Zero(w, h);
//		   sae_[1] = Eigen::MatrixXd::Zero(w, h);
}



void CompressiveForceGrasp::FTBiasedCallback(const geometry_msgs::WrenchStamped::ConstPtr &biased_ft_msg)
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



void CompressiveForceGrasp::gripperCallback(const baxter_core_msgs::EndEffectorState::ConstPtr &gripper_msg)
{
	Gripper_Percent=*gripper_msg;
}

void CompressiveForceGrasp::Davis_Slip_detection_Callback(const dvs_msgs::EventArray::ConstPtr &msg)
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




// Analysing callback packets
      for (const auto e : msg->events)
{
    	  double isharris= isCorner_Edge_HARRISi (e);   // Harris Detector

//    	  const int pol = e.polarity ? 1 : 0;
//    	  sae_[pol](e.x, e.y) = e.ts.toSec();

    	  count_raw =count_raw +1;
	  		if(e.polarity==0)
	  		{
	  			count_raw_positive = count_raw_positive + 1;
	  		}
	  		if(e.polarity==1)
	  		{
	  			count_raw_negative = count_raw_negative + 1;
	  		}

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
    	  	  	count_edge_positive = count_edge_positive + 1;
    	  	  		}
    	  	  		if(e.polarity==1)
    	  	  		{
    	  	    count_edge_negative = count_edge_negative + 1;
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
       	  	  	    	  	  	count_corner_positive = count_corner_positive + 1;
       	  	  	    	  	  		}
       	  	  	    	  	  		if(e.polarity==1)
       	  	  	    	  	  		{
       	  	  	    	  	  	count_corner_negative = count_corner_negative + 1;
       	  	  	    	  	  		}
    	  	  		 }


    	  	  	  if ((th1 > isharris) ||  (isharris < th2)) // Noise
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
		 t_grasp_start = h_clock::now();
		 grasp_monitor=true;
		 grasp_timer=true;
	}


		 if (grasp_monitor)

	{
		grasp_flat_frame = count_flat;
		grasp_edge_frame = count_edge;
		grasp_corner_frame = count_corner;
		grasp_raw_frame_events =count_raw;
		grasp_count_raw_positive=count_raw_positive; grasp_count_raw_negative=count_raw_negative;
		grasp_count_flat_positive =count_flat_positive; grasp_count_flat_negative =count_flat_negative;
		grasp_count_edge_positive=count_edge_positive; grasp_count_edge_negative=count_edge_negative;
		grasp_count_corner_positive =count_corner_positive; grasp_count_corner_negative =count_corner_negative;



	}


	if (grasp_stop)
		{
		 grasp_monitor=false;
		 grasp_start =false;
		 elaspedTime_grasp =std::chrono::duration_cast<std::chrono::milliseconds>(h_clock::now() - t_grasp_start).count();
		 grasp_duration_SS = elaspedTime_grasp;
		 grasp_timer=false;
		 grasp_stop =false;
		}




}



      flat_events_pub.publish(packets_noise);
      edge_events_pub.publish(packets_edge);
      corner_events_pub.publish(packets_corner);
      packets_edge.events.clear();
      packets_corner.events.clear();
      packets_noise.events.clear();


}

CompressiveForceGrasp::~CompressiveForceGrasp()
{
	    	  std::cout <<"Harris Threshold min----" <<min<< std::endl;
	    	  std::cout <<"Harris Threshold max----" <<max<< std::endl;

}



void CompressiveForceGrasp::run_controller_gripper()
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
//					{
//		if (grasp_start)
//		{
//
//			for (int i=0; i<=11; i++)
//					{
//
//    ros::Duration(5.0).sleep();
//
//    left_gripper.closeGripper();
//
//    ros::Duration(5.0).sleep();
//
//    left_gripper.openGripper();
//
//	 grasp_count = grasp_count+1;
//
//
//		}
//					}



//		while (ros::ok())
//				{
//	if (noise_sample_start)
//	{
//	    baxter_core_msgs::EndEffectorCommand command;
//	    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
//	    command.args = "{\"position\": 0.0}";
//	    command.id = 65538;
//
//	    // Send command several times to be safe
//	    for (std::size_t i = 0; i < 5; ++i)
//	    {
//	    	gripper_command_pub.publish(command);
//	      ros::Duration(2).sleep();
//	      ros::spinOnce();
//	    }
//	}
//
//	if (noise_sample_stop)
//		{
//		baxter_core_msgs::EndEffectorCommand command;
//		    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
//		    command.args = "{\"position\": 100.0}";
//		    command.id = 65538;
//
//		    // Send command several times to be safe
//		    for (std::size_t i = 0; i < 5; ++i)
//		    {
//		    	gripper_command_pub.publish(command);
//
//		      ros::Duration(2).sleep();
//		      ros::spinOnce();
//		    }
//		}
//				}
//	while (ros::ok())
//			{
//		baxter_core_msgs::EndEffectorCommand command;
//		    command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
//		    command.args = "{\"position\": 100.0}";
//		    command.id = 65538;
//			}
//}
}

bool CompressiveForceGrasp::ServiceCallback(object_test::Request  &req,object_test::Response &res)
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

bool CompressiveForceGrasp::ServiceCallbacksample(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{
	  std::cout<<" Samnple_start_Service" << std::endl;

		 count_raw=0; count_flat=0; count_corner=0; count_edge=0;
		 count_raw_positive=0; count_raw_negative=0;
	     count_flat_positive=0; count_flat_negative=0;
	     count_edge_positive=0; count_edge_negative=0;
	     count_corner_positive=0; count_corner_negative=0;
	     raw_max=0; flat_raw_max =0; edge_raw_max =0; corner_raw_max =0;
	     raw_max_N=0; flat_raw_max_N=0; edge_raw_max_N=0; corner_raw_max_N=0;
	     raw_max_P=0; flat_raw_max_P=0; edge_raw_max_P=0; corner_raw_max_P=0;
	     grasp_duration_SS=0;
	  noise_sample_start= true;
	  noise_sample_stop= false;

	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"Samnple_stop_Service" << std::endl;

	  noise_sample_stop= true;
	  noise_sample_start= false;

	res.status =1;


}
  return true;
}



void CompressiveForceGrasp::publish_frame_millisec(){
	// collecting complete datset


	Data.header=head_data;

	Data.gripper_material=gripper_material;
	Data.Grip_Holding_value=Grip_Holding_value;
	Data.object_size= object_size;
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

	Data.ft_raw_data = FT_Biased;
	Data.f_max_data = f_max_data;
	Data.t_max_data = t_max_data;

//	std_msgs::Float64MultiArray m;
//	   m.layout.dim.resize(2);
//	  m.layout.dim[0].stride = sae_[1].rows() * sae_[1].cols();
//	  m.layout.dim[0].size = sae_[1].rows();
//	  m.layout.dim[1].stride = sae_[1].cols();
//	  m.layout.dim[1].size = sae_[1].cols();
//	  if ((int)m.data.size() != sae_[1].size())
//	    m.data.resize(sae_[1].size());
//	  int ii = 0;
//
//	  for (int i = 0; i < sae_[1].rows(); ++i)
//	  {
//	    for (int j = 0; j < sae_[1].cols(); ++j)
//	    {
//	      m.data[ii++] = sae_[1].coeff(i, j);
//	    }
//	  }
//
//
//	  std_msgs::Float64MultiArray m2;
//	  	   m2.layout.dim.resize(2);
//	  	  m2.layout.dim[0].stride = sae_[2].rows() * sae_[2].cols();
//	  	  m2.layout.dim[0].size = sae_[2].rows();
//	  	  m2.layout.dim[1].stride = sae_[2].cols();
//	  	  m2.layout.dim[1].size = sae_[2].cols();
//	  	  if ((int)m2.data.size() != sae_[2].size())
//	  	    m2.data.resize(sae_[2].size());
//	  	  int ff = 0;
//
//	  	  for (int i = 0; i < sae_[2].rows(); ++i)
//	  	  {
//	  	    for (int j = 0; j < sae_[2].cols(); ++j)
//	  	    {
//	  	      m2.data[ff++] = sae_[2].coeff(i, j);
//	  	    }
//	  	  }


	  	Data.state_data=Gripper_Percent;
//	  	Data.raw_P=m;
//	  	Data.raw_N=m2;


	complete_data.publish(Data);
//	   sae_[0] = Eigen::MatrixXd::Zero(w, h);
//	   sae_[1] = Eigen::MatrixXd::Zero(w, h);
	 count_raw=0; count_flat=0; count_corner=0; count_edge=0;
	 count_raw_positive=0; count_raw_negative=0;
     count_flat_positive=0; count_flat_negative=0;
     count_edge_positive=0; count_edge_negative=0;
     count_corner_positive=0; count_corner_negative=0;
	f_max_data_compare=0;  t_max_data_compare=0;
}





}
