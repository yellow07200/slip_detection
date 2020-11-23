/*
 * ObjectDetection.cpp
 *
 *  Created on: Mar 25, 2019
 *      Author: raj
 */

#include "ObjectDetection.h"
namespace slip_detection_davis
{
ObjectDetection::ObjectDetection() {
	// TODO Auto-generated constructor stub
	  // allocate SAE matrices
	object_contour_pub = pnh_.advertise<dvs_msgs::EventArray>("/object_extraction", 1);
	noise_events_sub  = pnh_.subscribe("/dvs_noise_events", 0, &ObjectDetection::NoiseEventsCallback, this);
	edge_events_sub   = pnh_.subscribe("/dvs_edge_events", 0, &ObjectDetection::EdgeEventsCallback, this);
	corner_events_sub = pnh_.subscribe("/dvs_corner_events", 0, &ObjectDetection::CornerEventsCallback, this);

	sae_edge_[0] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_ );
	sae_edge_[1] = Eigen::MatrixXd::Zero(sensor_width_, sensor_height_ );
	//  std::cout<< " constructor " << std::endl;


	goal_service = pnh_.advertiseService("grasp_command", &ObjectDetection::ServiceCallback, this);
	grasp_start= false;
	grasp_stop= false;
}

ObjectDetection::~ObjectDetection() {
	// TODO Auto-generated destructor stub
}

bool ObjectDetection::ServiceCallback(object_test::Request  &req,object_test::Response &res)
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



void ObjectDetection::EdgeEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
	 // std::cout<< " calll " << std::endl;

	  dvs_msgs::EventArray opackets_edge;
	  opackets_edge.header = msg->header;
	  opackets_edge.width = msg->width;
	  opackets_edge.height = msg->height;
	for (const auto es : msg->events)
	       {
		  if (es.x < 4 || es.x >= sensor_width_-4 ||
				  es.y < 4 || es.y >= sensor_height_-4)
		  {
//			  std::cout<<"ddd" << std::endl;

		  }
		  else
		  {
//				  std::cout<< es.x << std::endl;
//				  std::cout<< es.y << std::endl;
if (grasp_start)
{
	//grasp_stop=false;

	  const int pol = es.polarity ? 1 : 0;
// sae_edge_[pol](es.x, es.y) = es.ts.toSec();
 sae_edge_[1](es.x, es.y) = es.ts.toSec();
		  opackets_edge.events.push_back(es);
		  //std::cout<<"grasp_start" << std::endl;

}
 if (grasp_stop){
	//grasp_start=false;
	 //cv::Mat events_positive(sae_edge_[0].rows(),sae_edge_[0].cols(),CV_32FC1,sae_edge_[0].data());
	 //cv::Mat events_negative(sae_edge_[1].rows(),sae_edge_[1].cols(),CV_8UC3,sae_edge_[1].data());
	 cv::Mat events_negative(sae_edge_[1].rows(),sae_edge_[1].cols(),CV_32F,sae_edge_[1].data());
	 Mat B; //storing the 1D index result
	 Mat C = events_negative.reshape(1, 1); //as mentioned by Michael Burdinov
	 sortIdx(C, B, SORT_EVERY_ROW + SORT_ASCENDING);
	 for (int i = 0; i < B.cols; i++) //from index 0 to index rows * cols of the original image
	 {
	     int val =  B.at<int>(0, i); //access B, which is in format CV_32SC1
	     int y = val / events_negative.cols; //convert 1D index into 2D index
	     int x = val % events_negative.cols;
	     std::cout << "idx " << val << " at (x/y) " << x << "/" << y
	               << " is " << (int) events_negative.at<uchar>(y, x) << endl;
	 }




//	 cv::Mat indices;
////	 cv::sortIdx(events_negative, indices, cv::SORT_EVERY_ROW | cv::SORT_DESCENDING);
//	 cv::sortIdx(events_negative, indices, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
//
//	 		int i,j;
//	 		int num = 0;
//	 		for(i=0; i<sensor_width_;i++)
//	 		 {
//	 			 for(j=0; j<sensor_height_;j++)
//	 			 {
//	 		 std::cout<< std::setprecision(15)<<indices.at<double>(i,j) << std::endl;
//	 		 cout<<row<<"\t"<<col<<"\n";
//	 		 if (num==50)
//	 		 {
//	 			 break;
//	 		 }
//	 		    num++;
//	 			 }
//	 		 }
//	 	    std::cout << "number of iterations: %d\n"<< num << std::endl;

//	 cv::Mat t1= cvCreateMat(events_negative.rows* events_negative.cols, 1, CV_32F);
//	 t1 = events_negative.reshape(1,events_negative.rows*events_negative.cols);
//
//
//	 cv::Mat ind= cvCreateMat(events_negative.rows*events_negative.cols,1,CV_16U);
//	 sortIdx(t1, ind,  CV_SORT_EVERY_COLUMN | CV_SORT_DESCENDING);
//
//	 for(int i=0;i<ind.rows;i++)
//	 {
//	     cout<<ind.at<ushort>(i,0)<<"\t\t"<<t1.at<float>(i,0)<<"\n";
//	 }
//
//	 for(int i=0;i<ind.rows;i++)
//	 {
//	     int row = ind.at<ushort>(i,0);
//	     int col = ind.at<ushort>(i,0);
//	     row = row /events_negative.cols;
//	     col = col% events_negative.cols;
//
//	     cout<<row<<"\t"<<col<<"\n";
//	 }



//	 	   Mat unsorted(1, 5, CV_32F);
//	 	   unsorted.at<int>(0, 0) = 40;
//	 	   unsorted.at<int>(0, 1) = 30;
//	 	   unsorted.at<int>(0, 2) = 100;
//	 	   unsorted.at<int>(0, 3) = 110;
//	 	   unsorted.at<int>(0, 4) = 10;
//
//	 	   Mat sorted;
//	 	   cv::sortIdx(unsorted, sorted, SORT_EVERY_ROW + SORT_ASCENDING);
//
//	 	   cout << sorted.at<int>(0, 0) << " " << sorted.at<int>(0, 1) << " " << sorted.at<int>(0, 2) << " " << sorted.at<int>(0, 3) << " " << sorted.at<int>(0, 4) << " " << endl;




//	 cv::Mat image_sort = cv::Mat::zeros(rect.height, rect.width, rect.type()); // allocated memory
//	 image(roi).copyTo(image_sort); // copy data in image_sorted
//	 std::sort(image_sort.data, image_sort.dataend); // call std::sort
//	 cv::Mat vectorized = image_sort.reshape(1, 1); // reshaped your WxH matrix into a 1x(W*H) vector.
	 // sorting
//	 int (*pm)[sensor_width_]= sae_edge_[1];
//	 int *pi = (int*)pm ;
//	 //Search
//

//
//	    // print it as an array of 9 int
//	    std::cout << "---------------------\n" ;
//	    for( int i = 0 ; i < (sensor_width_* sensor_height_ ); ++i ) std::cout << pi[i] << ' ' ;
//	    std::cout << '\n' ;

//	 cv::Mat events_both(sae_[2].rows(),sae_[2].cols(),CV_32FC1,sae_[2].data());
//	int i,j;
//	for(i=0; i<100;i++)
//	 {
//		 for(j=0; j<100;j++)
//		 {
//	 std::cout<< std::setprecision(15)<<sae_edge_[1](i, j) << std::endl;
//		 }
//	 }
	 String windowName = "Object_contour1"; //Name of the window
	 cv::namedWindow(windowName, CV_WINDOW_FREERATIO ); // Create a window

	 imwrite("object_edges.jpg", events_negative);
	 cv::imshow(windowName, events_negative); // Show our image inside the created window.
	 cv::waitKey(0); // Wait for any keystroke in the window
	 cv::destroyWindow(windowName); //destroy the created window
//ros::spinOnce();
}

//		  double et =es.ts.toSec();
//		  				int ex =es.x;
//		  				int ey =es.y;
//		  				bool ep =es.polarity;
//		  			  sae_edge_[ep](ex, ey) = et;
//		  		  opackets_edge.events.push_back(es);

		  }
		  //std::cout<< " Total time [ns]: " << std::endl;

	       }
	object_contour_pub.publish(opackets_edge);
}


 void ObjectDetection::contour_extraction(){
//	 Eigen::Matrix eigMat;
	  // update SAE
//	  const int pol = e.polarity ? 1 : 0;
//	  sae_[pol](e.x, e.y) = e.ts.toSec();
//	 cv::imshow("test", cv::eigen2cv(sae_[0]));
//	 cv::eigen2cv(sae_[0])
//	  	Mat img6 = imread("image.png");

// extract recent 50 timestamp from SAE









//	 cv::Mat events_positive(sae_edge_[0].rows(),sae_edge_[0].cols(),CV_32FC1,sae_edge_[0].data());
//	 cv::Mat events_negative(sae_edge_[1].rows(),sae_edge_[1].cols(),CV_8UC3,sae_edge_[1].data());
////	 cv::Mat events_both(sae_[2].rows(),sae_[2].cols(),CV_32FC1,sae_[2].data());
//
//	 String windowName = "Object_contour"; //Name of the window
//	 cv::namedWindow(windowName); // Create a window
//	 cv::imshow(windowName, events_negative); // Show our image inside the created window.
//	 cv::waitKey(0); // Wait for any keystroke in the window
//	 cv::destroyWindow(windowName); //destroy the created window

//	 String windowName1 = "Object_contour1"; //Name of the window
//	 cv::namedWindow(windowName1); // Create a window
//	 cv::imshow(windowName1, events_negative); // Show our image inside the created window.
//	 cv::waitKey(0); // Wait for any keystroke in the window
//	 cv::destroyWindow(windowName1); //destroy the created window
//	  //	ShowManyImages("Image", 3, events_positive,events_negative,events_both);

// 1. Assign a eigen matrix  value in opencv mat format

// 2. show the image

// 3.

}



 void ObjectDetection::NoiseEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
 {

 }
 void ObjectDetection::CornerEventsCallback(const dvs_msgs::EventArray::ConstPtr &msg)
 {

 }


// void ObjectDetection::ShowManyImages(string title, int nArgs, ...) {
// int size;
// int i;
// int m, n;
// int x, y;
//
// // w - Maximum number of images in a row
// // h - Maximum number of images in a column
// int w, h;
//
// // scale - How much we have to resize the image
// float scale;
// int max;
//
// // If the number of arguments is lesser than 0 or greater than 12
// // return without displaying
// if(nArgs <= 0) {
//     printf("Number of arguments too small....\n");
//     return;
// }
// else if(nArgs > 14) {
//     printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
//     return;
// }
// // Determine the size of the image,
// // and the number of rows/cols
// // from number of arguments
// else if (nArgs == 1) {
//     w = h = 1;
//     size = 300;
// }
// else if (nArgs == 2) {
//     w = 2; h = 1;
//     size = 300;
// }
// else if (nArgs == 3 || nArgs == 4) {
//     w = 2; h = 2;
//     size = 300;
// }
// else if (nArgs == 5 || nArgs == 6) {
//     w = 3; h = 2;
//     size = 200;
// }
// else if (nArgs == 7 || nArgs == 8) {
//     w = 4; h = 2;
//     size = 200;
// }
// else {
//     w = 4; h = 3;
//     size = 150;
// }
//
// // Create a new 3 channel image
// Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);
//
// // Used to get the arguments passed
// va_list args;
// va_start(args, nArgs);
//
// // Loop for nArgs number of arguments
// for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
//     // Get the Pointer to the IplImage
//     Mat img = va_arg(args, Mat);
//
//     // Check whether it is NULL or not
//     // If it is NULL, release the image, and return
//     if(img.empty()) {
//         printf("Invalid arguments");
//         return;
//     }
//
//     // Find the width and height of the image
//     x = img.cols;
//     y = img.rows;
//
//     // Find whether height or width is greater in order to resize the image
//     max = (x > y)? x: y;
//
//     // Find the scaling factor to resize the image
//     scale = (float) ( (float) max / size );
//
//     // Used to Align the images
//     if( i % w == 0 && m!= 20) {
//         m = 20;
//         n+= 20 + size;
//     }
//
//     // Set the image ROI to display the current image
//     // Resize the input image and copy the it to the Single Big Image
//     Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
//     Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
//     temp.copyTo(DispImage(ROI));
// }
//
// // Create a new window, and show the Single Big Image
// namedWindow( title, 1 );
// imshow( title, DispImage);
// waitKey(0);
//
// // End the number of arguments
// va_end(args);
// }

}
