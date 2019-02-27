#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/*
ros::Publisher pub;



void imgCallback(const sensor_msgs::ImageConstPtr& input){

	if(!input){
		return;
	}

	std::cout << input << "\n";
}
*/

/*
void cloudCallback (const sensor_msgs::PointCloud2ConstPtr input)
{

	// Create a container for the data
	sensor_msgs::PointCloud2 output;
		
	// Do data processing here
	output = *input;

	pcl::PointCloud<pcl::PointXYZRGB> cloud_RGB;
	
	pcl::fromROSMsg(output, cloud_RGB);
	
	//pcl::toROSMsg(cloud_RGB, output);



	// Publish the data
	pub.publish(output);

	for(int i = 0; i < cloud_RGB.points.size(); i++){

		pcl::PointXYZRGB p = cloud_RGB.points[i];

		//if(p.r != 0 && p.g != 0 && p.b != 0){
			std::cout << "Red: " << p.r << " Green: " << p.g << " Blue: " << p.b << "\n";
		//}

		if(p.x != 0 && p.y != 0 && p.z != 0){
			std::cout << "X: " << p.x << " Y: " << p.y << " Z: " << p.z << "\n";
		}
	}
}
*/

/*
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, imgCallback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
*/