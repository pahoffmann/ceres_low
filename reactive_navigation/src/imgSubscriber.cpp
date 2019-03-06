#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <image_transport/image_transport.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/array.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


using namespace std;
using namespace cv;
using namespace pcl;





Mat srcImage;
Mat irImage;
Mat depthImage;
Mat splitImage;
ros::Publisher pub;
int minLineLength = 100;
int maxLineGap = 50;
float max_angle = 10;

bool hasSeenLines = false;

std::deque<Vec2d> last3Lines; //we need to pop front



float calcAngleBetweenVectors(Vec2d v1, Vec2d v2){
	auto scalar = v1.dot(v2);
	float norm1 = cv::norm(v1);
	float norm2 = cv::norm(v2);

	return acos(scalar/(norm1 * norm2)) * (180/M_PI);
}


void disableEmitter(){

	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config config;

	double_param.name = "Emitter_Enabled";
	double_param.value = 0;
	config.doubles.push_back(double_param);
	
	srv_req.config = config;

	ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


    srcImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    splitImage = cv_bridge::toCvShare(msg, "bgr8")->image;

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
        cv::Vec2d result = v2 - v1;

        if(result[1] > 0) {
            result = result * -1;
        }

        double angle = calcAngleBetweenVectors(result, baseVec);

        double dotP = result.dot(baseVec);

        if(angle > 90){
            angle = 180 - angle;
        }

        if(angle > 20 && angle < 60)
        {
            //cv::line(srcImage, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 5);

            if(result[0] > 0){
                leftLines.push_back(line);
            }
            else{
                rightLines.push_back(line);
            }
        }      
    }

    if(leftLines.size() > 0 && rightLines.size() > 0){
    	hasSeenLines = true;
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


	    cv::Vec4i centerLine(320, 480, centerUp[0], centerUp[1]);

	    cv::line(srcImage, cv::Point(centerLine[0], centerLine[1]), cv::Point(centerLine[2], centerLine[3]), cv::Scalar(0, 0, 255), 5);


		//imshow("Color Image", srcImage);

		//TODO: calc angle, publish data

		Vec2d vertical(0, -1);
		Vec2d middle = centerUp - Vec2d(320,480);

		//save center line
		if(last3Lines.size() == 3){
			last3Lines.pop_front();
		}
		last3Lines.push_back(middle);

		angle = calcAngleBetweenVectors(vertical, middle);


		/*
		* Calculate the accel param, which will be between 0 and 1. For small angles it will tend to 1, for bigger angles, it will tend to 0
		* This way, the acceleration is limited by the current position in the field
		*/
		auto accel = ( std::sin(((angle * M_PI) / max_angle)) + 1) / 2;



		cout << "Acceleration: " << accel << endl;

		geometry_msgs::Twist twistMsg;



		if(angle > 10){

			if(middle[0] < 0){
				twistMsg.angular.z = 0.1 * accel;
			} 

			else if(middle[0] > 0) {
				twistMsg.angular.z = -0.1 * accel;
			} 
			cout << "Angle to big! I'll better turn around.." << endl;
		}
		else{

			twistMsg.linear.x = accel * 0.3;

			if(middle[0] < 0){
				twistMsg.angular.z = 0.25 * accel;
			} 

			else if(middle[0] > 0) {
				twistMsg.angular.z = -0.25 * accel;
			} 
			else {
				twistMsg.angular.z = 0;
			}
		}

		pub.publish(twistMsg);

    } else{
    	//no line seen :( 

    	geometry_msgs::Twist twistMsg;

    	if(hasSeenLines){

    		float angleCurrentlyNoLineSeen;
    		Vec2d averageVec(0,0);


    		for(Vec2d vec : last3Lines){
    			angleCurrentlyNoLineSeen += calcAngleBetweenVectors(vec, Vec2d(0, -1));
    			averageVec +=  vec;
    		}

    		angleCurrentlyNoLineSeen /=  last3Lines.size();

    		divide(last3Lines.size(), averageVec, averageVec);
    		//averageVec /= last3Lines.size();

    		float accel = (std::sin(((angleCurrentlyNoLineSeen * M_PI) / max_angle)) + 1) / 2;

    		twistMsg.linear.x = 0.3 * accel;

    		if(averageVec[0] < 0){
				twistMsg.angular.z = 0.15 * accel;
			} 

			else if(averageVec[0] > 0) {
				twistMsg.angular.z = -0.15 * accel;
			} 
			else {
				twistMsg.angular.z = 0;
			}



    		twistMsg.linear.x = 0.3;

    		pub.publish(twistMsg);

    	} else {

    		twistMsg.angular.z = 0.15;

    		pub.publish(twistMsg);
    	}
    	
    }

	//angle calculation done

	waitKey(30);


}



PointCloud<PointXYZRGB>::Ptr CreatePointCloud(Mat depth_image, Mat color_image, vector<float> K){


	PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
	pointcloud->reserve(depthImage.rows * depthImage.cols);
   	//vector<Point3f> points;

    float fx, fy, cx, cy;
    fx = (float)K[0];
    fy = (float)K[4];
    cx = (float)K[2];
    cy = (float)K[5];

    //cout << cx << " | " << cy << " | " << fx << " | " << fy << endl;

    for (int v = 0; v < depth_image.rows; v++){
    	for (int u = 0; u < depth_image.cols; u++){
    		float depth_value = depth_image.at<float>(v, u);

    		PointXYZRGB p;
    		p.x = ((u - cx)*depth_value*(1.0f / fx)) / 1000.0f;
    		p.y = ((v - cy)*depth_value*(1.0f / fy)) / 1000.0f;
    		p.z = (depth_value) / 1000.0f;
    		pointcloud->points.push_back(p);
    	}
    }
    return pointcloud;
}


void infoCallback(const sensor_msgs::CameraInfo &camera_info){

	Mat undistortedSplit;
	Mat undistortedDepth;

	vector<float> kValues, dValues;
	auto K = camera_info.K;
	auto D = camera_info.D;
	kValues.assign(K.begin(), K.end());
	//dValues.assign(D.begin(), D.end());

	if(!splitImage.empty() && !depthImage.empty() && !irImage.empty()){

		//undistort(splitImage, undistortedSplit, kValues, dValues);
		//undistort(depthImage, undistortedDepth, kValues, dValues);

		auto pointcloud = CreatePointCloud(depthImage, splitImage, kValues);
		sensor_msgs::PointCloud2 out;
		pcl::toROSMsg(*pointcloud, out);
		out.header.frame_id = "camera_link";

		//pointcloud_publisher.publish(out);

	}


}


void depthCallback(const sensor_msgs::ImageConstPtr& msg){

	depthImage = cv_bridge::toCvShare(msg, "16UC1")->image;

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


		double x;
	    double y;
		double length;
		double angle;



	    // convert to HSV color space
	    Mat hsvImage;
	    cvtColor(newHSL, hsvImage, CV_BGR2HSV);

	    // split the channels
	    std::vector<cv::Mat> hsvChannels;
	    split(hsvImage, hsvChannels);

	    // hue channels tells you the color tone, if saturation and value aren't too low.

	    int hueValue = 0; // green color
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
	        cv::Vec2d result = v2 - v1;

	        if(result[1] > 0) {
	            result = result * -1;
	        }

	        double dotP = result.dot(baseVec);

	        double angle = std::acos(dotP / cv::norm(result)) * (180/M_PI);

	        if(angle > 90){
	            angle = 180 - angle;
	        }

	        if(angle > 20 && angle < 60)
	        {
	            //cv::line(srcImage, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0, 255, 0), 5);

	            if(result[0] > 0){
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

	    cv::line(newHSL, cv::Point(rightMeanLine[0], rightMeanLine[1]), cv::Point(rightMeanLine[2], rightMeanLine[3]), cv::Scalar(255, 0, 0), 5);
	    cv::line(newHSL, cv::Point(leftMeanLine[0], leftMeanLine[1]), cv::Point(leftMeanLine[2], leftMeanLine[3]), cv::Scalar(255, 0, 0), 5);

	    cv::Vec2d centerDown((leftMeanLine[0] + rightMeanLine[2]) / 2, leftMeanLine[1]);
	    cv::Vec2d centerUp((leftMeanLine[2] + rightMeanLine[0]) / 2, leftMeanLine[3]);


	    cv::Vec4i centerLine(320, 0, centerUp[0], centerUp[1]);

	    cv::line(newHSL, cv::Point(centerLine[0], centerLine[1]), cv::Point(centerLine[2], centerLine[3]), cv::Scalar(0, 0, 255), 5);


    	//imshow("Infrared Image", irImage);
    	//imshow("NDVI", newHSL);
    	waitKey(30);
  	}
}




int main(int argc, char **argv)
{  

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    disableEmitter();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
    //image_transport::Subscriber sub2 = it.subscribe("camera/infra1/image_rect_raw", 1, infraCallback);
    //image_transport::Subscriber sub3 = it.subscribe("camera/aligned_depth_to_color/image_raw", 1, depthCallback);
	//ros::Subscriber sub4 = nh.subscribe("camera/color/camera_info", 1, infoCallback);
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::spin();

    return 0;

}