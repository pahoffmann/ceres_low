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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>



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
    sor.setLeafSize (0.01, 0.01, 0.01);
    sor.filter (cloud_filtered);


    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud_filtered, point_cloud);
    pcl::copyPointCloud(point_cloud, *point_cloudPtr);


    //passthrough filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (point_cloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*filtered_cloud);

    pcl::copyPointCloud(*filtered_cloud, *point_cloudPtr);


    //Plane model segmentation
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

    int i = 0, nr_points = (int) filtered_cloud->points.size ();
    // While 30% of the original cloud is still there
    while (filtered_cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (filtered_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (filtered_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        filtered_cloud.swap (cloud_f);
        i++;
    }


    /*
    //Object segmentation
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(filtered_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.04);
    ec.setMinClusterSize(100); //100
    ec.setMaxClusterSize(25000);//99000000
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

    int j= 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZRGB point;
            point.x = filtered_cloud->points[*pit].x;
            point.y = filtered_cloud->points[*pit].y;
            point.z = filtered_cloud->points[*pit].z;

            if (j == 0) //Red	#FF0000	(255,0,0)
            {
                point.r = 0;
                point.g = 0;
                point.b = 255;
            }
            else if (j == 1) //Lime	#00FF00	(0,255,0)
            {
                point.r = 0;
                point.g = 255;
                point.b = 0;
            }
            else if (j == 2) // Blue	#0000FF	(0,0,255)
            {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }
            else if (j == 3) // Yellow	#FFFF00	(255,255,0)
            {
                point.r = 255;
                point.g = 255;
                point.b = 0;
            }
            else if (j == 4) //Cyan	#00FFFF	(0,255,255)
            {
                point.r = 0;
                point.g = 255;
                point.b = 255;
            }
            else if (j == 5) // Magenta	#FF00FF	(255,0,255)
            {
                point.r = 255;
                point.g = 0;
                point.b = 255;
            }
            else if (j == 6) // Olive	#808000	(128,128,0)
            {
                point.r = 128;
                point.g = 128;
                point.b = 0;
            }
            else if (j == 7) // Teal	#008080	(0,128,128)
            {
                point.r = 0;
                point.g = 128;
                point.b = 128;
            }
            else if (j == 8) // Purple	#800080	(128,0,128)
            {
                point.r = 128;
                point.g = 0;
                point.b = 128;
            }
            else
            {
                if (j % 2 == 0)
                {
                    point.r = 255 * j / (cluster_indices.size());
                    point.g = 128;
                    point.b = 50;
                }
                else
                {
                    point.r = 0;
                    point.g = 255 * j / (cluster_indices.size());
                    point.b = 128;
                }
            }
            point_cloud_segmented->push_back(point);

        }
        j++;
    }
    std::cerr<< "segemnted:  " << (int)point_cloud_segmented->size() << "\n";
    std::cerr<< "origin:     " << (int)filtered_cloud->size() << "\n";
    // Convert to ROS data type
    point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
    if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
    else pcl::toPCLPointCloud2(*filtered_cloud, cloud_filtered);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
    pub.publish (output);
     */




  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*filtered_cloud, output);

  pub.publish (output);

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
