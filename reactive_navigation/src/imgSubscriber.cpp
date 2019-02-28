#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>



int minLineLength = 175;
int maxLineGap = 100;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


    double x;
    double y;
	double length;
	double angle;


    cv::Mat srcImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    // convert to HSV color space
    cv::Mat hsvImage;
    cv::cvtColor(srcImage, hsvImage, CV_BGR2HSV);

    // split the channels
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsvImage, hsvChannels);

    // hue channels tells you the color tone, if saturation and value aren't too low.

    int hueValue = 50; // green color
    int hueRange = 15; // how much difference from the desired color we want to include to the result If you increase this value, for example a red color would detect some orange values, too.

    int minSaturation = 50;
    int minValue = 50; 

	// [hue, saturation, value]
    cv::Mat hueImage = hsvChannels[0]; 

    // is the color within the lower hue range?
    cv::Mat hueMask;
    cv::inRange(hueImage, hueValue - hueRange, hueValue + hueRange, hueMask);



    // if desired color is near the border of the hue space, check the other side too:
    /*if (hueValue - hueRange < 0 || hueValue + hueRange > 180)
    {
        cv::Mat hueMaskUpper;
        int upperHueValue = hueValue + 180; // in reality this would be + 360 instead
        cv::inRange(hueImage, upperHueValue - hueRange, upperHueValue + hueRange, hueMaskUpper);

        // add this mask to the other one
        hueMask = hueMask | hueMaskUpper;
    }
	*/

    // filter out all the pixels where saturation and value do not fit the limits:
    cv::Mat saturationMask = hsvChannels[1] > minSaturation;
    cv::Mat valueMask = hsvChannels[2] > minValue;

    hueMask = (hueMask & saturationMask) & valueMask;

    cv::imshow("Desired Color Only", hueMask);

    // perform the line detection
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(hueMask, lines, 1, CV_PI / 360, 50, minLineLength, maxLineGap);

    // draw the result as big green lines:
    for (unsigned int i = 0; i < lines.size(); ++i)
    {
    	cv::Vec4i line = lines[i];


    	//line vector components
   		x = line[2] - line[0];
    	y = line[3] - line[1];

    	cv::Vec2d v1(line[0], line[1]);
    	cv::Vec2d v2(line[2], line[3]);
    	cv::Vec2d baseVec(1, 0);
    	cv::Vec2d res = v2 - v1;

    	if(res[1] > 0) {
    		res = res * -1;
    	}

    	double dotP = res.dot(baseVec);

    	double angle = std::acos(dotP / cv::norm(res)) * (180/3.141592);

        if(angle > 90){
        	angle = 180 - angle;
        }

   		if(angle > 20 && angle < 60)
    	{
    		cv::line(srcImage, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 5);	
    	}      
    }


    cv::imshow("Source Image", srcImage);
    cv::waitKey(30);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);

  ros::spin();

  return 0;

}