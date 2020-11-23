/*
 * slip_data_test.cpp
 *
 *  Created on: Feb 13, 2019
 *      Author: raj
 */


#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <cstdlib>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
//moveit packages
#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayTrajectory.h>

//boost bind to use multiple arguments
#include "boost/bind.hpp"
#include "boost/shared_ptr.hpp"
#include "boost/ref.hpp"

/////////////////////////////////////////////// INCLUDES


//ROS

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//filters, registration
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/shot.h>
#include <pcl/features/3dsc.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

//visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/time.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudDP;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudDRP;
using namespace ros;
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)

{
// 1.Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud and save it
// and convert back to sensor msg and visualize in rviz

sensor_msgs::PointCloud2 cloudmsg = *input;
PointCloudDRP::Ptr cloudpcl (new PointCloudDRP);
pcl::fromROSMsg (cloudmsg,*cloudpcl);
int count = 0;
if(count<=2) {
pcl::io::savePCDFileASCII ("scene.pcd", *cloudpcl);
	ROS_INFO("saved scene");
}


pcl::toROSMsg(*cloudpcl,cloudmsg);
sensor_msgs::PointCloud2 output= cloudmsg;

/*
sensor_msgs::PointCloud2 cloudmsgs;
          PointCloudDP::Ptr object_reg_pc (new PointCloudDP);

          pcl::io::loadPCDFile ("final_pose_object_align.pcd", *object_reg_pc);
          ROS_INFO("conversion to sensor_msgs publishes the object in object_output");
          pcl::toROSMsg(*object_reg_pc,cloudmsgs);
          sensor_msgs::PointCloud2 object_output= cloudmsgs;
          pub.publish(object_output);
//pub.publish(output);
  */

++count;
}
int main(int argc, char **argv)
{
	ros::init(argc,argv,"scene_generation");
	ros::NodeHandle n;
	//  ros::Publisher pub;

	// Create a ROS subscriber for the input point cloud
		  ros::Subscriber sub = n.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
  		//pub = n.advertise<sensor_msgs::PointCloud2> ("object_output", 1);

		  // Create a ROS publisher for the output point cloud
		  //pub = n.advertise<sensor_msgs::PointCloud2> ("output", 1);

	ros::spin();
	return 0;
	}

