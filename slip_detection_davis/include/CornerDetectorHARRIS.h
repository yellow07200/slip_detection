/*
 * CornerDetectorHARRIS.h
 *
 *  Created on: Apr 7, 2019
 *      Author: raj
 */

#ifndef SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORHARRIS_H_
#define SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORHARRIS_H_

#include <deque>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
//#include "davisdataprocessing.h"
#include <softhandslipdetection.h>
#include <dvs_msgs/Event.h>

#include "harris_local_event_queues.h"
#include "harris_distinct_queue.h"
//#include <CompressiveForceGrasp.h>
#include <softhandslipdetection.h>
//#include "SlipDetectionRawTest.h"

namespace slip_detection_davis
{
//class CornerDetector_HARRIS : public  CompressiveForceGrasp{

class CornerDetector_HARRIS : public  soft_hand_slip_detection{
public:
	CornerDetector_HARRIS(  );

//	CornerDetector_HARRIS( ros::NodeHandle* nodehandle );
	virtual ~CornerDetector_HARRIS();

	   std::pair<bool,bool> isCorner_Edge_HARRIS (const dvs_msgs::Event &e, double th_E, double th_C);
	   double isCorner_Edge_HARRISi (const dvs_msgs::Event &e);
	    bool isCorner (const dvs_msgs::Event &e);
//	    bool isCornerFAST (const dvs_msgs::Event &e);
//	    bool isCornerARC (const dvs_msgs::Event &e);


private:
		  HarrisLocalEventQueues* queues_;
		  ros::NodeHandle pnh_;

	   Eigen::MatrixXd Gkernel, Sob_G_x, sob_x; // Guassian Kernel, sobel opertor
	  // const Eigen::MatrixXi local_patch; //9x9 local patch
	   Eigen::MatrixXd dx, dy; //  // Gradient operated on the patch
	   Eigen::MatrixXi window_;
	  static const int sensor_width_ = 240;
	  static const int sensor_height_ = 180;
	  // contains one event
	  struct QueueEvent
	  {
	    int prev, next;
	    int x, y;
	  };
	  std::vector<QueueEvent> queue_;
	  double harris_threshold_;
	  // parameters
	  int queue_size_;
	  int window_size_;
	  int kernel_size_;
	 // std::vector<CornerDetector_HARRIS> queues_;

//	  int first_, last_;
//	  int queue_max_;
	  int factorial(int n) const;
	  int pasc(int k, int n) const;
//	  void addNew(int x, int y);
//	  bool isFull() const;
//	  int getIndex(int x, int y, bool polarity) const;
//	  Eigen::MatrixXi getWindow() const;

};

}
#endif /* SLIP_DETECTION_DAVIS_SRC_CORNERDETECTORHARRIS_H_ */
