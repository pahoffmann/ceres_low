#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <stdlib.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h> //transform between coordinate systems
//#include <librealsense2/h/rs_sensor.h>





using namespace std;
using namespace cv;
using namespace rs2;


Mat srcImage;
Mat irImage;
int minLineLength = 175;
int maxLineGap = 100;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


    srcImage = cv_bridge::toCvShare(msg, "bgr8")->image;


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

    cv::imshow("Desired Color Only", hueMask);

    // perform the line detection
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(hueMask, lines, 1, CV_PI / 360, 50, minLineLength, maxLineGap);

    std::vector<cv::Vec4i> leftLines;
    std::vector<cv::Vec4i> rightLines;

    // draw the result as big green lines:
    for (unsigned int i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i line = lines[i];


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



    cv::imshow("Source Image", srcImage);
    cv::waitKey(30);

}


void infraCallback(const sensor_msgs::ImageConstPtr& msg){

	irImage = cv_bridge::toCvShare(msg, "8UC1")->image;

	/*cv::Mat bgrChannel[3];
	cv::split(srcImage, bgrChannel);

	cv::Mat ndviImage1 = (irImage - bgrChannel[2]); 
    cv::Mat ndviImage2 = (irImage + bgrChannel[2]);
    cv::Mat res;
    cv::divide(ndviImage2,ndviImage1, res);

    cv::Mat newHue = (res.clone()) * (180); // float 32
    //cv::Mat newLigthness = cv::Mat::ones(irImage.rows, irImage.cols, CV_32FC1);
    //cv::Mat newSaturation = cv::Mat::ones(irImage.rows, irImage.cols, CV_32FC1);*/


    //Just testing some stuff here..
    /*Mat newHSL;
    if(!srcImage.empty()){
        cvtColor(srcImage,srcImage,COLOR_HSV2BGR);
        std::vector<Mat> channels;
        split(srcImage, channels);
        Mat numeratorMat = channels[2] - channels[0]; // red - blue
        Mat denominatorMat = channels[2] + channels[0]; // red + blue
        Mat ratio;
        cv::divide(denominatorMat, numeratorMat, ratio, 1., 5); // here 5 specifies type of ratio (CV_32FC1)

        Mat newHue = (ratio.clone() - 0.5 ) * (-360); // float 32
        Mat newLigthness = Mat::ones(srcImage.rows, srcImage.cols, CV_32FC1);
        Mat newSaturation = Mat::ones(srcImage.rows, srcImage.cols, CV_32FC1);

        vector<Mat> newChannels;
        newChannels.push_back(newHue);
        newChannels.push_back(newSaturation);
        newChannels.push_back(newLigthness);
        cv::merge(newChannels, newHSL); 

        normalize(ratio, ratio, 0, 255, NORM_MINMAX, CV_8UC1);
        //or: normalize(ratio, ratio, 127,127,NORM_L2,CV_8UC1);
        // Apply a color map 
        applyColorMap(ratio, newHSL, COLORMAP_HSV);
    }*/
    


	if(!irImage.empty() && !srcImage.empty()){

        //todo:: crop image

        Rect rectCrop(106, 89, 400,300); // don't change these values pls

        Mat cropped = irImage(rectCrop);
        Mat croppedNew(480, 640, CV_8UC1);

        resize(cropped, croppedNew, croppedNew.size(), 0, 0, CV_INTER_CUBIC);

        cvtColor(srcImage,srcImage,COLOR_HSV2BGR); //convert?

        /*for(int i = 0; i < srcImage.rows; i++){
            for(int j = 0; j < srcImage.rows; j++){
                std::cout << srcImage.at<double>(i,j) << "  ||  ";
            }
            std::cout << std::endl;
        }*/

        cv::Mat bgrChannel[3];
        cv::split(srcImage, bgrChannel);

        cv::Mat ndviImage1;
        cv::addWeighted(irImage, 1, bgrChannel[2], -1, 0.0, ndviImage1);// = (irImage - bgrChannel[2]); //numerator (IR - RED)
        cv::Mat ndviImage2;// = (irImage + bgrChannel[2]);
        cv::addWeighted(irImage, 1, bgrChannel[2], 1, 0.0, ndviImage2); //denominator (IR + RED)
        cv::Mat res;
        cv::divide(ndviImage1,ndviImage2, res); //denominator, numerator

        //normalize(res, res, 0, 255, NORM_MINMAX, CV_8UC1);
        //normalize(res, res, 127,127,NORM_L2,CV_8UC1);
        // Apply a color map

        res = (res * 90) + 90;

        /*for(int i = 0; i < res.rows; i++){
            for(int j = 0; j < res.rows; j++){
                std::cout << res.at<double>(i,j) << "  ||  ";
            }
            std::cout << std::endl;
        }*/

        //Mat newHSL;
        //applyColorMap(res, newHSL, COLORMAP_HSV);
        /*Mat color;
        cvtColor(newHSL,color,COLOR_HSV2BGR);*/
        cv::imshow("dsds", croppedNew);
		//cv::imshow("Infrared", irImage);
    	cv::imshow("Source Image", srcImage);
    	//cv::imshow("NDVI", res);
  	}


}

/*rs2_extrinsics getExtrinsics(){

    rs2::pipeline pipe;

    rs2::config config;
    config.enable_stream(RS_STREAM_COLOR, RS2_FORMAT_RGBA8);
    config.enable_stream(RS_STREAM_INFRARED);



}*/

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;


    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
    image_transport::Subscriber sub2 = it.subscribe("camera/infra1/image_rect_raw", 1, infraCallback);



    ros::spin();


    return 0;

}