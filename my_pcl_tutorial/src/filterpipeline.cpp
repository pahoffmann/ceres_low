#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_types_conversion.h>


ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  //Convert sensor_msgs to pcl::pointcloud<t>
  pcl::PointCloud<pcl::PointXYZRGB> cloud_RGB;
  pcl::fromROSMsg (*input, cloud_RGB);
  //std::cout<<cloud_RGB.points[1].;
    /*
    for(int i = 0; i < cloud_RGB.points.size(); i++)
    {
        pcl::PointXYZRGB p = cloud_RGB.points[i];
        if(p.rgb > 0.00)printf("%f", p.rgb);

        uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;

        //std::cout << "x: "<< p.x <<"  y: "<< p.y << "  z: " << p.z << std::endl;
        //if(p.rgb != 0.0) std::cout<<"rgb:"<<p.rgb<<std::endl;
       //std::cout<<"rgb:"<<p.r<<" "<<p.g<<" "<<p.b<<std::endl;
        //std::cout<<"rgb: "<<rgb<<std::endl;
        //if (i<2)
        if(i<1) std::cout<<p;
        break;

    }
    */


  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  //pcl::PointCloud<pcl::PointXYZRGB> cloudcp (cloud_rgb);

  //pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
  /* pcl::PointCloud<pcl::PointXYZHSV> cloud_hsv;
  pcl::PointCloudXYZRGBtoXYZHSV(cloud_RGB, cloud_hsv);

  std::cout  << "cloud size(): " << cloud_hsv.points.size() << std::endl;
  for(int i = 0; i < cloud_hsv.points.size(); i++)
  {
    pcl::PointXYZHSV p = cloud_hsv.points[i];
    std::cout << "x: "<< p.x <<"  y: "<< p.y << "  z: " << p.z << "  h: " << p.h << "  s: " << p.s << "  v: " << p.v << std::endl;
  }*/

/*
  pcl::PointCloud<PointRGB>::Ptr rgb_cloud (new pcl::PointCloud<PointRGB>);

  rgb_cloud->width  = 4; // sample data
  rgb_cloud->height = 1;
  rgb_cloud->is_dense = false;
  rgb_cloud->points.resize (rgb_cloud->width * rgb_cloud->height);

  uint8_t r = 255, g = 0, b = 0; // red color
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

  for (int i = 0; i < rgb_cloud->points.size (); i++)
  {
    rgb_cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    rgb_cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    rgb_cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    rgb_cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  for(int i = 0; i < rgb_cloud->points.size(); i++)
  {
    PointRGB p = rgb_cloud->points[i];
    std::cout << "x: "<< p.x <<"  y: "<< p.y << "  z: " << p.z << "  rgb: " << p.rgb << std::endl;
  }
  */


  //pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
//  pcl::PointXYZRGBtoXYZHSV(*cloud, *hsvcloud);



/*
  //Convert to sensor_msgs
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_filtered, output);
*/
  // Publish the data.
  //pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
