/*
 * object_extraction_control.cpp
 *
 *  Created on: Apr 23, 2019
 *      Author: raj
 */




#include "ros/ros.h"
#include <slip_detection_davis/object_test.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_command");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<slip_detection_davis::object_test>("grasp_command");
  slip_detection_davis::object_test srv;
  srv.request.event_capture_command = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  ros::spin();

  return 0;
}
