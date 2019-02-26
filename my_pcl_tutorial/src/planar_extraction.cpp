//
// Created by hiep on 26.02.19.
//

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>



ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

    //Convert sensor_msgs to pcl::pointcloud<t>
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::fromROSMsg (*input, *cloud_RGB);
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;



    pcl_conversions::toPCL(*input, *cloud);


    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (cloud_filtered);


    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (cloud_filtered, *cloud_xyz);



    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    int i = 0, nr_points = (int) cloud_xyz->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_xyz->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_xyz);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_xyz);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_xyz.swap (cloud_f);
        i++;
    }




    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_xyz, output);

    pub.publish (output);




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
