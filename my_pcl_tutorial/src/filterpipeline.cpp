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
#include <pcl/segmentation/region_growing_rgb.h>



ros::Publisher pub;


//Class
template <typename PointT>
class ConditionThresholdHSV : public pcl::ConditionBase<PointT>
{
public:
    typedef typename boost::shared_ptr<ConditionThresholdHSV<PointT> > Ptr;

    ConditionThresholdHSV (float min_h, float max_h, float min_s, float max_s, float min_v, float max_v) :
            min_h_(min_h), max_h_(max_h), min_s_(min_s), max_s_(max_s), min_v_(min_v), max_v_(max_v)
    {
        // Make min_h_ and max_h_ fall within [0, 360)
        assert (!std::isnan(min_h) && !std::isnan(max_h));
        while (min_h_ < 0) min_h_ += 360;
        while (min_h_ >= 360) min_h_ -= 360;
        while (max_h_ < 0) max_h_ += 360;
        while (max_h_ >= 360) max_h_ -= 360;
    }

    // Evaluate whether the color of the given point falls within the specified thresholds
    virtual bool evaluate(const PointT & p) const
    {
        float h, s, v;
        rgb2hsv (p.r, p.g, p.b, h, s, v);
        return (!std::isnan(h) && !std::isnan(s) && !std::isnan(v) &&
                ((min_h_ < max_h_) ? ((min_h_ <= h) && (h <= max_h_)) : ((min_h_ <= h) || (h <= max_h_))) &&
                (min_s_ <= s) && (s <= max_s_) &&
                (min_v_ <= v) && (v <= max_v_));
    }

    void rgb2hsv (uint8_t r, uint8_t g, uint8_t b, float & h, float & s, float & v) const
    {
        float maxval = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
        float minval = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
        float minmaxdiff = maxval - minval;

        if (maxval == minval)
        {
            h = 0;
            s = 0;
            v = maxval;
            return;
        }
        else if (maxval == r)
        {
            h = 60.0*((g - b)/minmaxdiff);
            if (h < 0) h += 360.0;
        }
        else if (maxval == g)
        {
            h = 60.0*((b - r)/minmaxdiff + 2.0);
        }
        else // (maxval == b)
        {
            h = 60.0*((r - g)/minmaxdiff + 4.0);
        }
        s = 100.0 * minmaxdiff / maxval;
        v = maxval;
    }

protected:
    float min_h_, max_h_, min_s_, max_s_, min_v_, max_v_;
};


double MedianCalc(std::vector<double> koord)
{
    size_t size = koord.size();

    if (size == 0)
    {
        return 0;  // Undefined, really.
    }
    else
    {
        sort(koord.begin(), koord.end());
        if (size % 2 == 0)
        {
            return (koord[size / 2 - 1] + koord[size / 2]) / 2;
        }
        else
        {
            return koord[size / 2];
        }
    }
}

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
    sor.setLeafSize (0.007, 0.007, 0.007);
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

    //colorfilter
    pcl::ConditionalRemoval<pcl::PointXYZRGB> removal_filter;
    removal_filter.setKeepOrganized (false);
    ConditionThresholdHSV<pcl::PointXYZRGB>::Ptr condition (new ConditionThresholdHSV<pcl::PointXYZRGB> (-20,20, 75,100, 25,255));
    removal_filter.setCondition (condition);

    removal_filter.setInputCloud (filtered_cloud);
    removal_filter.filter (*point_cloudPtr);

    /*
    std::vector<double> x_koord;
    std::vector<double> y_koord;

    for (int i = 0; i < point_cloudPtr->points.size (); i++)
    {
        x_koord.push_back(point_cloudPtr->points[i].x);
        y_koord.push_back(point_cloudPtr->points[i].y);
    }

    double x_median= MedianCalc(x_koord);
    double y_median= MedianCalc(y_koord);

    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_koord(new pcl::PointCloud<pcl::PointXYZRGB>);
    final_koord->width  = 1;
    final_koord->height = 1;
    final_koord->points.resize (final_koord->width * final_koord->height);

    // Generate the data

    final_koord->points[0].x = x_median;
    final_koord->points[0].y = y_median;
    final_koord->points[0].z = 0.0;
    final_koord->points[0].r = 255;
    final_koord->points[0].g = 0;
    final_koord->points[0].b = 0;

    std::cout<<final_koord->points[0].x<<std::endl;
    std::cout<<final_koord->points[0].y<<std::endl;

    */



    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*point_cloudPtr, output);

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
