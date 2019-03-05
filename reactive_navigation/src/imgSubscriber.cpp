#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h> //transform between coordinate systems
#include <librealsense2/h/rs_sensor.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


using namespace std;
using namespace cv;
using namespace rs2;



void disableEmitter(){

	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config config;

	double_param.name = "Emitter_Enabled";
	double_param.value = false;
	config.doubles.push_back(double_param);
	
	srv_req.config = config;

	ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);

}

Mat srcImage;
Mat irImage;
Mat splitImage;
int minLineLength = 175;
int maxLineGap = 100;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


    srcImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    splitImage = cv_bridge::toCvShare(msg, "bgr8")->image;

    //srcImage.convertTo(srcImage, CV_32F);


    double x;
    double y;
	double length;
	double angle;



    // convert to HSV color space
    Mat hsvImage;
    cvtColor(srcImage, hsvImage, CV_BGR2HSV);

    // split the channels
    std::vector<cv::Mat> hsvChannels;
    split(hsvImage, hsvChannels);

    // hue channels tells you the color tone, if saturation and value aren't too low.

    int hueValue = 50; // green color
    int hueRange = 15; // how much difference from the desired color we want to include to the result If you increase this value, for example a red color would detect some orange values, too.

    int minSaturation = 50;
    int minValue = 50; 

    // [hue, saturation, value]
    Mat hueImage = hsvChannels[0]; 

    // is the color within the lower hue range?
    Mat hueMask;
    cv::inRange(hueImage, hueValue - hueRange, hueValue + hueRange, hueMask);



    // if desired color is near the border of the hue space, check the other side too:
    if (hueValue - hueRange < 0 || hueValue + hueRange > 180)
    {
        cv::Mat hueMaskUpper;
        int upperHueValue = hueValue + 180; // in reality this would be + 360 instead
        cv::inRange(hueImage, upperHueValue - hueRange, upperHueValue + hueRange, hueMaskUpper);

        // add this mask to the other one
        hueMask = hueMask | hueMaskUpper;
    }
    

    // filter out all the pixels where saturation and value do not fit the limits:
    Mat saturationMask = hsvChannels[1] > minSaturation;
    Mat valueMask = hsvChannels[2] > minValue;

    hueMask = (hueMask & saturationMask) & valueMask;

    // perform the line detection
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(hueMask, lines, 1, CV_PI / 360, 50, minLineLength, maxLineGap);

    std::vector<cv::Vec4i> leftLines;
    std::vector<cv::Vec4i> rightLines;

    // draw the result as big green lines:
    for (unsigned int i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i line = lines[i];


        cv::Vec2d v1(line[0], line[1]); //x1 and y1
        cv::Vec2d v2(line[2], line[3]); //x2 and y2
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
            //cv::line(srcImage, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 5);

            if(res[0] > 0){
                leftLines.push_back(line);
            }
            else{
                rightLines.push_back(line);
            }
        }      
    }

    Vec4i rightMeanLine(0,0,0,0);
    Vec4i leftMeanLine(0,0,0,0);
    if(rightLines.size() > 0 && leftLines.size() > 0){
        for(auto line : rightLines){
            rightMeanLine = rightMeanLine + line;
        }
        rightMeanLine = rightMeanLine / (int)rightLines.size();

        for(auto line : leftLines){
            leftMeanLine = leftMeanLine + line;
        }
        leftMeanLine = leftMeanLine / (int)leftLines.size();
    }

    cv::line(srcImage, cv::Point(rightMeanLine[0], rightMeanLine[1]), cv::Point(rightMeanLine[2], rightMeanLine[3]), cv::Scalar(255, 0, 0), 5);
    cv::line(srcImage, cv::Point(leftMeanLine[0], leftMeanLine[1]), cv::Point(leftMeanLine[2], leftMeanLine[3]), cv::Scalar(255, 0, 0), 5);

    cv::Vec2d centerDown((leftMeanLine[0] + rightMeanLine[2]) / 2, leftMeanLine[1]);
    cv::Vec2d centerUp((leftMeanLine[2] + rightMeanLine[0]) / 2, leftMeanLine[3]);


    cv::Vec4i centerLine(centerDown[0],centerDown[1], centerUp[0], centerUp[1]);

    cv::line(srcImage, cv::Point(centerLine[0], centerLine[1]), cv::Point(centerLine[2], centerLine[3]), cv::Scalar(0, 0, 255), 5);

    cv::imshow("Green-Detection", srcImage);

}


void infraCallback(const sensor_msgs::ImageConstPtr& msg){

	irImage = cv_bridge::toCvShare(msg, "8UC1")->image;
  
	if(!irImage.empty() && !splitImage.empty()){

        Mat res, cropped;
        Mat numerator, denominator, newHSL;
        Mat bgrChannel[3];

        Rect rectCrop(116, 89, 400,300);

        cropped = irImage(rectCrop);
        Mat croppedNew(480, 640, CV_8UC1);

        resize(cropped, croppedNew, croppedNew.size(), 0, 0, CV_INTER_CUBIC);

        irImage = croppedNew;

        split(splitImage, bgrChannel);

        addWeighted(irImage, 1, bgrChannel[2], -1, 0.0, numerator, CV_32F);
        addWeighted(irImage, 1, bgrChannel[2], 1, 0.0, denominator, CV_32F);
        divide(numerator, denominator, res);


        normalize(res, res, 0, 255, NORM_MINMAX);
        res.convertTo(res, CV_8UC1);

        applyColorMap(res, newHSL, COLORMAP_JET);
    	//imshow("Infrared Image", irImage);
    	imshow("NDVI", newHSL);
    	waitKey(30);
  	}


}

/*rs2_extrinsics getExtrinsics(){

    rs2::pipeline pipe;

    rs2::config config;
    config.enable_stream(rs2_stream::RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);


}*/


int main(int argc, char **argv)
{  

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
	disableEmitter();


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
    image_transport::Subscriber sub2 = it.subscribe("camera/infra1/image_rect_raw", 1, infraCallback);
    ros::spin();

    return 0;

}