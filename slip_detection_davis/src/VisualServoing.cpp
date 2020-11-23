/*
 * VisualServoing.cpp
 *
 *  Created on: Jan 27, 2020
 *      Author: user
 */

#include "VisualServoing.h"
namespace slip_detection_davis
{
Visual_Servoing::Visual_Servoing() {

	// Subscribers
    davis_sub_ = pnh_.subscribe("/dvs/events", 0, &Visual_Servoing::Davis_feature_Callback, this);

    //Pubishers
    centtoid_pub = pnh_.advertise<dvs_msgs::EventArray>("/object_center", 1);

        noise_events_pub = pnh_.advertise<dvs_msgs::EventArray>("/dvs_noise_events_soft", 1);
        edge_events_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_edge_events_soft", 1);
        corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/dvs_corner_events_soft", 1);
        complete_data =pnh_.advertise<slip_detection_davis::SlipDetectionDataComplete_analysis_final3>("/slip_complete_data", 1);

    //ROS service
    	goal_service = pnh_.advertiseService("grasp_command", &Visual_Servoing::ServiceCallback1, this);
    	sample_service = pnh_.advertiseService("sample_command", &Visual_Servoing::ServiceCallback2, this);

		  sae= Eigen::MatrixXd::Zero(sensor_width_,sensor_height_);

}




void Visual_Servoing::Davis_feature_Callback(const dvs_msgs::EventArray::ConstPtr &msg)
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
	  packets_raw.header = msg->header;
	  packets_raw.width = msg->width;
	  packets_raw.height = msg->height;

// Analysing callback packets
      for (const auto e : msg->events)
{
    	  double isharris= isCorner_Edge_HARRISi (e);   // Harris Detector
//    	  bool fast= isCornerFAST(e); // Fast corner detector
//    	  bool arc= isCornerARC(e);   // ARC corner detector

    	  count_raw =count_raw +1;



// Separating edge from the detector
    	  	  	   if (isharris <= th1) // Edge %0.01
    	  	  	     {
       	  	  		count_edge = count_edge+1;
E_e=e;

    	  	  		   packets_edge.events.push_back(e);
//    	  		  		if (isharris < min)
//    	  		  		{
//    	  		  		    min = isharris;
//    	  		  		   // minPos = count;
//    	  		  		}
//    	  	  		if(e.polarity==0)
//    	  	  		{
//    	  	  	count_edge_positive = count_edge_positive+1;
//    	  	  		}
//    	  	  		if(e.polarity==1)
//    	  	  		{
//    	  	    count_edge_negative = count_edge_negative+1;
//    	  	  		}
//         	    	  std::cout <<"Edge----" <<count_edge<< std::endl;

    	  	  	     }

// Separating corner from the detector

        	  	   if (isharris >= th2) // corner
    	  	  		 {

          	  	  		count_corner = count_corner+1;
          	  	  	C_e=e;
    	  	  		   packets_corner.events.push_back(e);
//    	  	  		if (isharris > max)
//    	  	  		    {
//    	  	  		    	max = isharris;
//    	  	  		    	// minPos = count;
//    	  	  		    }
////       	    	  std::cout <<"Corner----" <<count_corner<< std::endl;
//
//       	  	  	if(e.polarity==0)
//       	  	  	    	  	  		{
//       	  	  	    	  	  	count_corner_positive = count_corner_positive+1;
//       	  	  	    	  	  		}
//       	  	  	    	  	  		if(e.polarity==1)
//       	  	  	    	  	  		{
//       	  	  	    	  	  	count_corner_negative = count_corner_negative+1;
//       	  	  	    	  	  		}
    	  	  		 }

// Separating flat events from the detector

    	  	  	  if ((isharris > 0.1) &&  (isharris < th2)) // Noise
    	  	  		{
       	  	  		count_flat = count_flat+1;

    	  	  		   packets_noise.events.push_back(e);
////             	    	  std::cout <<"Noise----" <<count_noise<< std::endl;
//
//          	  	  	if(e.polarity==0)
//          	  	  	       	  	  	    	  	  		{
//          	  	 	count_flat_positive = count_flat_positive+1;
//          	  	          	  	  	       	  	  	    	  	  		}
//          	  	          	  	  	       	  	  	    	  	  		if(e.polarity==1)
//          	  	          	  	  	       	  	  	    	  	  		{
//          	  	          	  	  	       	  	  	    	  	  	count_flat_negative = count_flat_negative+1;
//          	  	          	  	  	       	  	  	    	  	  		}
          	  	  	    	  	  		 }






}



      noise_events_pub.publish(packets_noise);
      edge_events_pub.publish(packets_edge);
      corner_events_pub.publish(packets_corner);
      packets_edge.events.clear();
      packets_corner.events.clear();
      packets_noise.events.clear();


}



  void Visual_Servoing::function1()
  {

	  sae(C_e.x, C_e.y) = C_e.ts.toSec();
//	  dvs_msgs::EventArray packets_raw;
//	  packets_raw.header = msg->header;
//	  packets_raw.width = msg->width;
//	  packets_raw.height = msg->height;
//
//	  dvs_msgs::EventArray centre;
//	  C_c
  }


  void Visual_Servoing::function2()
  {

  }



  void Visual_Servoing::function3()
  {

  }


  void Visual_Servoing::publish_data()
  {

  }








bool Visual_Servoing::ServiceCallback1(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{	  std::cout<<"Sampling_start Service 1" << std::endl;


	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"Sampling_stop Service 2" << std::endl;


	res.status =1;


}
  return true;
}

bool Visual_Servoing::ServiceCallback2(object_test::Request  &req,object_test::Response &res)
{
if (req.event_capture_command ==0)
{
	  std::cout<<"Grasp_start Service 3" << std::endl;


	res.status =1;

}
if (req.event_capture_command ==1)
{
	  std::cout<<"Grasp_stop Service 4" << std::endl;


	res.status =1;


}
  return true;
}

Visual_Servoing::~Visual_Servoing() {
	// TODO Auto-generated destructor stub
}

}
