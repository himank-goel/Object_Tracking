#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = 5;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud);

  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  // viewer.showCloud(ptrCloud);
  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  viewer.addPointCloud(ptrCloud);

  while (!viewer.wasStopped())
  {
  }

  for (size_t i = 0; i < cloud.points.size(); ++i)
    std::cout << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  return (0);
}