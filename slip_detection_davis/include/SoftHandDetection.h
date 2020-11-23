/*
 * SoftHandDetection.h
 *
 *  Created on: Jun 9, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_SOFTHANDDETECTION_H_
#define SLIP_DETECTION_DAVIS_SRC_SOFTHANDDETECTION_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <iostream>

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <slip_detection_davis/object_test.h>

#include <geometry_msgs/WrenchStamped.h>

#include <iostream>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

namespace slip_detection_davis
{
class Soft_Hand_Detection {
public:
	 Soft_Hand_Detection();
	 virtual ~Soft_Hand_Detection();
	//callbacks
	  void eNoiseEventsCallback(const dvs_msgs::Event::ConstPtr &msg);
	  void eEdgeEventsCallback(const dvs_msgs::Event::ConstPtr &msg);
	  void eCornerEventsCallback(const dvs_msgs::Event::ConstPtr &msg);
	  void NoiseEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  void EdgeEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  void CornerEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  void SlipDetectionExperiment();
	  void FTBiasedCallback(const geometry_msgs::WrenchStamped bias_ft);
	  void FTRawCallback(const geometry_msgs::WrenchStamped raw_ft);
//	  void Soft_Hand_Detection::FTRawCallback(const baxter_core_msgs::EndEffectorState gripper_left);

	// ROS service
	  bool ServiceCallback(object_test::Request  &req,object_test::Response &res);

	// Global Variables
	  dvs_msgs::EventArray noise;
	  dvs_msgs::EventArray edge;
	  dvs_msgs::EventArray corner;
	  dvs_msgs::Event enoise;
	  dvs_msgs::Event eedge;
	  dvs_msgs::Event ecorner;
      ros::ServiceServer goal_service;
	  bool grasp_start, grasp_stop, slip_detect;
      int max_edge_events, raw_noise_events_;
      double total_time_;
      int total_events_, total_corners_;

private:
	  Eigen::MatrixXd edge_frame[2];

	  ros::Publisher  object_contour_pub; // Publish classified online events
	  ros::Subscriber ft_bias_sub, ft_raw_sub, gripper_sub; // Subscribe data from Davis

	  ros::Subscriber noise_events_sub, edge_events_sub, corner_events_sub; // Subscribe data from Davis
	  ros::Subscriber enoise_events_sub, eedge_events_sub, ecorner_events_sub; // Subscribe data from Davis

	  ros::NodeHandle pnh_;
};
}
#endif /* SLIP_DETECTION_DAVIS_SRC_SOFTHANDDETECTION_H_ */
