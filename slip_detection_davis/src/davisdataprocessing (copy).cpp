/*
 * davisdataprocessing.cpp
 *
 *  Created on: Feb 17, 2019
 *      Author: raj
 */

#include "davisdataprocessing.h"

namespace slip_detection_davis
{
davis_data_processing::davis_data_processing(ros::NodeHandle* nodehandle) : pnh_(*nodehandle),total_time_(0.), total_events_(0), total_corners_(0)
,
circle3_ {{0, 3}, {1, 3}, {2, 2}, {3, 1},
           {3, 0}, {3, -1}, {2, -2}, {1, -3},
           {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
           {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
 circle4_ {{0, 4}, {1, 4}, {2, 3}, {3, 2},
           {4, 1}, {4, 0}, {4, -1}, {3, -2},
           {2, -3}, {1, -4}, {0, -4}, {-1, -4},
           {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
           {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}},kSmallCircle_{{0, 3}, {1, 3}, {2, 2}, {3, 1},
               {3, 0}, {3, -1}, {2, -2}, {1, -3},
               {0, -3}, {-1, -3}, {-2, -2}, {-3, -1},
               {-3, 0}, {-3, 1}, {-2, 2}, {-1, 3}},
     kLargeCircle_{{0, 4}, {1, 4}, {2, 3}, {3, 2},
               {4, 1}, {4, 0}, {4, -1}, {3, -2},
               {2, -3}, {1, -4}, {0, -4}, {-1, -4},
               {-2, -3}, {-3, -2}, {-4, -1}, {-4, 0},
               {-4, 1}, {-3, 2}, {-2, 3}, {-1, 4}}

//davis_data_processing::davis_data_processing(bool detect): total_time_(0.), total_events_(0), total_corners_(0)

{
	//FAST = new CornerDetector_Fast();
//	if(detect)
//	{
	test_pub  =pnh_.advertise<dvs_msgs::EventArray>("/dvs_eventss", 1);
	corner_events_pub =pnh_.advertise<dvs_msgs::EventArray>("/corner_eventss", 1);
    davis_sub_ = pnh_.subscribe("/dvs/events", 0, &davis_data_processing::DavisCallback, this);
    ROS_INFO("constructor davis_data_processing");
//	}

//    noise_events_pub=nh_.advertise<dvs_msgs::Event>("/noise_events", 1);
//    edge_events_pub =nh_.advertise<dvs_msgs::Event>("/edge_events", 1);


//    davis_sub_ = nh_.subscribe("/davis_left/events", 0, &davis_data_processing::LeftDavisCallback, this);
//    davis_sub_ = nh_.subscribe("/davis_right/events", 0, &davis_data_processing::RightDavisCallback, this);
//    right_events_pub = nh_.advertise<dvs_msgs::Event>("/right_davis_events", 1);
//    left_davis_sub_ = nh_.advertise<dvs_msgs::Event>("/left_davis_events", 1);

	  // allocate SAE matrices
	  fsae_[0] = Eigen::MatrixXd::Zero(fsensor_height_, fsensor_width_);
	  fsae_[1] = Eigen::MatrixXd::Zero(fsensor_height_, fsensor_width_);
	  // Initialize Surface of Active Events to 0-timestamp
	  asae_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
	  asae_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
	  asae_latest_[0] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
	  asae_latest_[1] = Eigen::MatrixXd::Zero(kSensorWidth_, kSensorHeight_);
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

	  dvs_msgs::EventArray packets_event;
	  packets_event.header = msg->header;
	  packets_event.width = msg->width;
	  packets_event.height = msg->height;
    //  ROS_INFO("callback");
	  utils::time::Timer<std::chrono::nanoseconds> timer;

      for (const auto e : msg->events)
       {
  	  //  ROS_INFO("ddddddddddd");
    	  //FAST->isCorner(e);
    		//davis_data_processing::isCorner(e);
    	  isNoise(e);
    	  isEdge(e);
    	  if (isCornerARC(e))
    	    {
    	  packets_corner.events.push_back(e);
    	    }
    	//    ROS_INFO("eveeeeeeee");
    	  packets_event.events.push_back(e);

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
  	//noise_events_pub.publish(packets_noise);
  	//edge_events_pub.publish(packets_edge);
  	corner_events_pub.publish(packets_corner);
  	test_pub.publish(packets_event);

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




bool davis_data_processing::isNoise(const dvs_msgs::Event &ex)
{

return true;
}


bool davis_data_processing::isEdge(const dvs_msgs::Event &ex)
{








	return true;

}





























bool davis_data_processing::isCornerFAST(const dvs_msgs::Event &ex)
{
	  //fsae_[0] = Eigen::MatrixXd::Zero(fsensor_height_+1, fsensor_width_+1);
	  //fsae_[1] = Eigen::MatrixXd::Zero(fsensor_height_+1, fsensor_width_+1);
	dvs_msgs::Event e= ex;
  // update SAE
  const int pol = e.polarity ? 1 : 0;
  fsae_[pol](e.x, e.y) = e.ts.toSec();

  const int max_scale = 1;

  // only check if not too close to border
  const int cs = max_scale*4;
  if (e.x < cs || e.x >= fsensor_width_-cs ||
      e.y < cs || e.y >= fsensor_height_-cs)
  {
    return false;
  }

  bool found_streak = false;

  for (int i=0; i<16; i++)
  {
    for (int streak_size = 3; streak_size<=6; streak_size++)
    {
      // check that streak event is larger than neighbor
      if (fsae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]) <  fsae_[pol](e.x+circle3_[(i-1+16)%16][0], e.y+circle3_[(i-1+16)%16][1]))
        continue;

      // check that streak event is larger than neighbor
      if (fsae_[pol](e.x+circle3_[(i+streak_size-1)%16][0], e.y+circle3_[(i+streak_size-1)%16][1]) <          fsae_[pol](e.x+circle3_[(i+streak_size)%16][0], e.y+circle3_[(i+streak_size)%16][1]))
        continue;

      double min_t = fsae_[pol](e.x+circle3_[i][0], e.y+circle3_[i][1]);
      for (int j=1; j<streak_size; j++)
      {
        const double tj = fsae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);
        if (tj < min_t)
          min_t = tj;
      }

      bool did_break = false;
      for (int j=streak_size; j<16; j++)
      {
        const double tj = fsae_[pol](e.x+circle3_[(i+j)%16][0], e.y+circle3_[(i+j)%16][1]);

        if (tj >= min_t)
        {
          did_break = true;
          break;
        }
      }

      if (!did_break)
      {
        found_streak = true;
        break;
      }

    }
    if (found_streak)
    {
      break;
    }
  }

  if (found_streak)
  {
    found_streak = false;
    for (int i=0; i<20; i++)
    {
      for (int streak_size = 4; streak_size<=8; streak_size++)
      {
        // check that first event is larger than neighbor
        if (fsae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]) <  fsae_[pol](e.x+circle4_[(i-1+20)%20][0], e.y+circle4_[(i-1+20)%20][1]))
          continue;

        // check that streak event is larger than neighbor
        if (fsae_[pol](e.x+circle4_[(i+streak_size-1)%20][0], e.y+circle4_[(i+streak_size-1)%20][1]) <          fsae_[pol](e.x+circle4_[(i+streak_size)%20][0], e.y+circle4_[(i+streak_size)%20][1]))
          continue;

        double min_t = fsae_[pol](e.x+circle4_[i][0], e.y+circle4_[i][1]);
        for (int j=1; j<streak_size; j++)
        {
          const double tj = fsae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj < min_t)
            min_t = tj;
        }

        bool did_break = false;
        for (int j=streak_size; j<20; j++)
        {
          const double tj = fsae_[pol](e.x+circle4_[(i+j)%20][0], e.y+circle4_[(i+j)%20][1]);
          if (tj >= min_t)
          {
            did_break = true;
            break;
          }
        }

        if (!did_break)
        {
          found_streak = true;
          break;
        }
      }
      if (found_streak)
      {
        break;
      }
    }
  }

  return found_streak;
}


bool davis_data_processing::isCornerARC(const dvs_msgs::Event &e) {
    // Update Surface of Active Events
	double et =e.ts.toSec();
	int ex =e.x;
	int ey =e.y;
	bool ep =e.polarity;

    const int pol = ep ? 1 : 0;
    const int pol_inv = (!ep) ? 1 : 0;
    double & t_last = asae_latest_[pol](ex,ey);
    double & t_last_inv = asae_latest_[pol_inv](ex, ey);

    // Filter blocks redundant spikes (consecutive and in short time) of the same polarity
    // This filter is required if the detector is to operate with corners with a majority of newest elements in the circles
    if ((et > t_last + filter_threshold_) || (t_last_inv > t_last) ) {
      t_last = et;
      asae_[pol](ex, ey) = et;
    } else {
      t_last = et;
      return false;
    }

    // Return if too close to the border
    const int kBorderLimit = 4;
    if (ex < kBorderLimit || ex >= (kSensorWidth_ - kBorderLimit) ||
        ey < kBorderLimit || ey >= (kSensorHeight_ - kBorderLimit)) {
      return false;
    }

    // Define constant and thresholds
    const int kSmallCircleSize = 16;
    const int kLargeCircleSize = 20;
    const int kSmallMinThresh = 3;
    const int kSmallMaxThresh = 6;
    const int kLargeMinThresh = 4;
    const int kLargeMaxThresh = 8;


    bool is_arc_valid = false;
    // Small Circle exploration
    // Initialize arc from newest element
    double segment_new_min_t = asae_[pol](ex+kSmallCircle_[0][0], ey+kSmallCircle_[0][1]);

    // Left and Right are equivalent to CW and CCW as in the paper
    int arc_right_idx = 0;
    int arc_left_idx;

    // Find newest
    for (int i=1; i<kSmallCircleSize; i++) {
      const double t =asae_[pol](ex+kSmallCircle_[i][0], ey+kSmallCircle_[i][1]);
      if (t > segment_new_min_t) {
        segment_new_min_t = t;
        arc_right_idx = i; // % End up in the maximum value
      }
    }
    // Shift to the sides of the newest element;
    arc_left_idx = (arc_right_idx-1+kSmallCircleSize)%kSmallCircleSize;
    arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
    double arc_left_value = asae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
    double arc_right_value = asae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
    double arc_left_min_t = arc_left_value;
    double arc_right_min_t = arc_right_value;

    // Expand
    // Initial expand does not require checking
    int iteration = 1; // The arc already contain the maximum
    for (; iteration<kSmallMinThresh; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
        }
        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = asae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if (arc_left_min_t < segment_new_min_t) {
          segment_new_min_t = arc_left_min_t;
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = asae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }
    int newest_segment_size = kSmallMinThresh;

    // Further expand until completion of the circle
    for (; iteration<kSmallCircleSize; iteration++) {
      // Decide the most promising arc
      if (arc_right_value > arc_left_value) { // Right arc
        // Include arc in new segment
        if ((arc_right_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1; // Check
          if (arc_right_min_t < segment_new_min_t) {
            segment_new_min_t = arc_right_min_t;
          }
        }

        // Expand arc
        arc_right_idx= (arc_right_idx+1)%kSmallCircleSize;
        arc_right_value = asae_[pol](ex+kSmallCircle_[arc_right_idx][0], ey+kSmallCircle_[arc_right_idx][1]);
        if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
          arc_right_min_t = arc_right_value;
        }
      } else { // Left arc
        // Include arc in new segment
        if ((arc_left_value >=  segment_new_min_t)) {
          newest_segment_size = iteration+1;
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }
        }

        // Expand arc
        arc_left_idx= (arc_left_idx-1+kSmallCircleSize)%kSmallCircleSize;
        arc_left_value = asae_[pol](ex+kSmallCircle_[arc_left_idx][0], ey+kSmallCircle_[arc_left_idx][1]);
        if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
          arc_left_min_t = arc_left_value;
        }
      }
    }

    if (// Corners with newest segment of a minority of elements in the circle
        // These corners are equivalent to those in Mueggler et al. BMVC17
            (newest_segment_size <= kSmallMaxThresh) ||
        // Corners with newest segment of a majority of elements in the circle
        // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the large circle
        ((newest_segment_size >= (kSmallCircleSize - kSmallMaxThresh)) && (newest_segment_size <= (kSmallCircleSize - kSmallMinThresh)))) {
      is_arc_valid = true;
    }

    // Large Circle exploration
    if (is_arc_valid) {
    is_arc_valid = false;

      segment_new_min_t = asae_[pol](ex+kLargeCircle_[0][0], ey+kLargeCircle_[0][1]);
      arc_right_idx = 0;

      // Initialize in the newest element
      for (int i=1; i<kLargeCircleSize; i++) {
        const double t =asae_[pol](ex+kLargeCircle_[i][0], ey+kLargeCircle_[i][1]);
        if (t > segment_new_min_t) {
          segment_new_min_t = t;
          arc_right_idx = i; // % End up in the maximum value
        }
      }
      // Shift to the sides of the newest elements;
      arc_left_idx = (arc_right_idx-1+kLargeCircleSize)%kLargeCircleSize;
      arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
      arc_left_value = asae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                 ey+kLargeCircle_[arc_left_idx][1]);
      arc_right_value = asae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                  ey+kLargeCircle_[arc_right_idx][1]);
      arc_left_min_t = arc_left_value;
      arc_right_min_t = arc_right_value;

      // Expand
      // Initial expand does not require checking
      iteration = 1;
      for (; iteration<kLargeMinThresh; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
          }
          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = asae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if (arc_left_min_t < segment_new_min_t) {
            segment_new_min_t = arc_left_min_t;
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = asae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                     ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }
      newest_segment_size = kLargeMinThresh;

      // Further expand until completion of the circle
      for (; iteration<kLargeCircleSize; iteration++) {
        // Decide the most promising arc
        if (arc_right_value > arc_left_value) { // Right arc
          // Include arc in new segment
          if ((arc_right_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_right_min_t < segment_new_min_t) {
              segment_new_min_t = arc_right_min_t;
            }
          }

          // Expand arc
          arc_right_idx= (arc_right_idx+1)%kLargeCircleSize;
          arc_right_value = asae_[pol](ex+kLargeCircle_[arc_right_idx][0],
                                      ey+kLargeCircle_[arc_right_idx][1]);
          if (arc_right_value < arc_right_min_t) { // Update minimum of the arc
            arc_right_min_t = arc_right_value;
          }
        } else { // Left arc
          // Include arc in new segment
          if ((arc_left_value >=  segment_new_min_t)) {
            newest_segment_size = iteration+1;
            if (arc_left_min_t < segment_new_min_t) {
              segment_new_min_t = arc_left_min_t;
            }
          }

          // Expand arc
          arc_left_idx= (arc_left_idx-1+kLargeCircleSize)%kLargeCircleSize;
          arc_left_value = asae_[pol](ex+kLargeCircle_[arc_left_idx][0],
                                    ey+kLargeCircle_[arc_left_idx][1]);
          if (arc_left_value < arc_left_min_t) { // Update minimum of the arc
            arc_left_min_t = arc_left_value;
          }
        }
      }

      if (// Corners with newest segment of a minority of elements in the circle
          // These corners are equivalent to those in Mueggler et al. BMVC17
              (newest_segment_size <= kLargeMaxThresh) ||
          // Corners with newest segment of a majority of elements in the circle
          // This can be commented out to decrease noise at expenses of less repeatibility. If you do, DO NOT forget to comment the equilvent line in the small circle
          (newest_segment_size >= (kLargeCircleSize - kLargeMaxThresh) && (newest_segment_size <= (kLargeCircleSize - kLargeMinThresh))) ) {
        is_arc_valid = true;
      }
    }

    return is_arc_valid;
}

}
