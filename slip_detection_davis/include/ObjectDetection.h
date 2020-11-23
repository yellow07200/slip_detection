/*
 * ObjectDetection.h
 *
 *  Created on: Mar 25, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_OBJECTDETECTION_H_
#define SLIP_DETECTION_DAVIS_SRC_OBJECTDETECTION_H_
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

//#include <ShowManyImages.h>
using namespace cv;
using namespace std;

namespace slip_detection_davis
{

class ObjectDetection {
public:


	ObjectDetection();
	virtual ~ObjectDetection();

	void contour_extraction();
//	void contour_extraction();

	 void ShowManyImages(string title, int nArgs, ...);
	  void NoiseEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  void EdgeEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  void CornerEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg);
	  bool ServiceCallback(object_test::Request  &req,object_test::Response &res);

	  bool grasp_start, grasp_stop;

		ros::ServiceServer goal_service;

private:
	  Eigen::MatrixXd sae_edge_[2];


	  ros::Publisher  object_contour_pub; // Publish classified online events
	  ros::Subscriber noise_events_sub, edge_events_sub, corner_events_sub; // Subscribe data from Davis
	  ros::NodeHandle pnh_;
  // SAE

  // pixels on circle
  int circle3_[16][2];
  int circle4_[20][2];

  // parameters
  static const int sensor_width_ = 240;
  static const int sensor_height_ = 180;
};
}
#endif /* SLIP_DETECTION_DAVIS_SRC_OBJECTDETECTION_H_ */
