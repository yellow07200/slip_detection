/*
 * obj_ext_test.cpp
 *
 *  Created on: May 28, 2019
 *      Author: raj
 */




#include <deque>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <Eigen/Eigen>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

//#include <dvs_msgs/EventArray.h>
//#include <dvs_msgs/Event.h>
//#include <davisdataprocessing.h>
//#include "CornerDetectorARC.h"
//#include "CornerDetectorFAST.h"
//#include "CornerDetectorHARRIS.h"

//#include "corner_event_detector/detector.h"
//#include "corner_event_detector/harris_detector.h"
//#include "corner_event_detector/fast_detector.h"
#include "object_extract.h"
#include "object_extract_emxAPI.h"
#include "object_extract_emxutil.h"
#include "object_extract_initialize.h"
#include "object_extract_terminate.h"

using namespace cv;
using namespace std;
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "object_extract");

	ros::NodeHandle nh_;
	 // slip_detection_davis::davis_data_processing* process(&nh_);
	  // load parameter

//	convert a C++ array into an 2D opencv Mat
//	double x[100][100];
//	cv::Mat A(100, 100, CV_64F, x);
//	double x[100][100];
//	cv::Mat A(100,100,CV_64F);
//	std::memcpy(A.data, x, 100*100*sizeof(double));

    std::cout << "hii"<< endl;

	cv::Mat image=cv::imread("myImage.jpeg");
    std::cout << "columns= " << image.cols << "Rows= " << image.rows << endl;
//    double xss[240][180];
    int a=240;
    int b=180;
    double yss[43200];
    double xss[43200];
    double image_mat[240][180];

    for(int i=0;i<image.cols;i++)
       {
          for(int j=0;j<image.rows;j++)
             {
        	  image_mat[i][j]=image.at<double>(j,i);
//        	  std::cout<< "i" <<","<<"j"<<image.at<double>(j,i) << std::endl;
             }
       }


    emxArray_real_T *border_x, *border_y;
border_x = emxCreateWrapper_real_T(xss, a,b);
border_y = emxCreateWrapper_real_T(yss, a, b);
object_extract(image_mat, border_x,border_y);
std::cout << "border_x= " << *border_x->size<< ", /n border_y= " << *border_y->size << endl;



	   cv::namedWindow("My Image", CV_WINDOW_FREERATIO);
	   cv::imshow("My Image",image);
	   cv::waitKey(0);
	   cv::destroyAllWindows();
//	 cv::Mat events_negative(sae_edge_[1].rows(),sae_edge_[1].cols(),CV_32F,sae_edge_[1].data());
//	 String windowName = "Object_contour1"; //Name of the window
//	 cv::namedWindow(windowName, CV_WINDOW_FREERATIO ); // Create a window
//
//	 imwrite("object_edges.jpg", events_negative);
//	 cv::imshow(windowName, events_negative); // Show our image inside the created window.
//	 cv::waitKey(0); // Wait for any keystroke in the window
//	 cv::destroyWindow(windowName); //destroy the created window
//	 object_extract();
//  ros::spin();


  return 0;
}
