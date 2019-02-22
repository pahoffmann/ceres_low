#include <iostream>
#include <pcl/io/io.h>
#include <pcl/point_types_conversion.h>

typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<PointRGB> PointCloudRGB;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

void create_point_cloud(PointCloudRGB::Ptr rgb_cloud)
{
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
}

void print_rgb_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud)
{
  std::cout << "rgb_cloud size(): " << rgb_cloud->points.size() <<
std::endl;
  for(int i = 0; i < rgb_cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p = rgb_cloud->points[i];
    std::cout << "x: "<< p.x <<"  y: "<< p.y << "  z: " << p.z << "  rgb: "
<< p.rgb << std::endl;
  }
}

void print_hsv_point_cloud(PointCloudHSV::Ptr hsv_cloud)
{
  std::cout  << "hsv_cloud size(): " << hsv_cloud->points.size() <<
std::endl;
  for(int i = 0; i < hsv_cloud->points.size(); i++)
  {
    PointHSV p = hsv_cloud->points[i];
    std::cout << "x: "<< p.x <<"  y: "<< p.y << "  z: " << p.z << "  h: " <<
p.h << "  s: " << p.s << "  v: " << p.v << std::endl;
  }
}

int main (int argc, char** argv)
{
  PointCloudRGB::Ptr rgb_cloud (new PointCloudRGB);
  PointCloudHSV::Ptr hsv_cloud (new PointCloudHSV);

  create_point_cloud(rgb_cloud);
  pcl::PointCloudXYZRGBtoXYZHSV(*rgb_cloud, *hsv_cloud);

  print_rgb_point_cloud(rgb_cloud);
  print_hsv_point_cloud(hsv_cloud);
  return 0;
}
