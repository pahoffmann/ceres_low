#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>


/**
 * This program picks up  an image broadcasted by 'imgBroadcast.cpp'
 * and applies canny edge detection to that image. 
 *
 * The user can manually set the lower threshold for the canny edge detector
 * by using a trackbar
 *
 * If applied the canny detector generates a mask
 * (bright lines representing the edges on a black background)
 */
using namespace cv;
using namespace std;

// Global Variables

Mat src, src_gray, bgr;
Mat dst, cdst, detected_edges;
Mat Bands[3],merged;

int lowThreshold = 50;
int ratio = 3;
int kernel_size = 3;
int minLineLength = 125;
int maxLineGap = 5;



/**
 *	@function CannyThreshold
 *  @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

/*
void CannyThreshold(int, void*)
{
	// Reduce noise with a kernel 3x3
	blur(src_gray, detected_edges, cv::Size(3,3));

	// Canny detector
	Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

	// Using Canny's output as a mask, we display the result
	dst = Scalar::all(0);

	// Copy source image to dst image
	src.copyTo(dst, detected_edges);


	//Show the image
	imshow("Edge Map", dst);
  	waitKey(30);

}

*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  // Load image
  src = cv_bridge::toCvShare(msg, "bgr8")->image;

  split(src, Bands);
  vector<Mat> channels = {Bands[0], 3*Bands[1]-Bands[0]-Bands[2], Bands[2]};
  merge(channels, merged);



  

  // Create a matrix of the same type and size as the image (for dst)
  //dst.create(src.size(), src.type());

  //Use Canny Edge Detection on source image
  Canny(src, dst, lowThreshold, lowThreshold*ratio, kernel_size);


  // Create colour image from gray image for hough transform
  cvtColor(dst, cdst, CV_GRAY2BGR);

/**
 * HOUGH TRANFORMATION
 * 
 * @function HoughLinesP() returns an array of (rho, theta)
 * @val rho is measured in pixels
 * @val theta is measures in radians
 *
 * HoughlinesP(inputImage as binary image(use canny edge detection to create binary),
 *				rho accuracy,
 *				theta accuracy,
 *				threshold: minimum accuracy an element needs to be considered a line),
 *				minLineLength: elements shorter than this are rejected
 *				maxLineGap: maximum allowed gap between line segments to treat them as a single line
 *
 */


	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 80, minLineLength, maxLineGap);
	for(size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), kernel_size, CV_AA);
	}

	imshow("Source Image", src);
	imshow("Detected Lines", cdst);
	imshow("Merged", merged);

	waitKey(30);
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  ros::spin();


}