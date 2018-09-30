#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../Pcd_Logs/2018-09-26_19-46-31.939/cloud000020.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-10.0, 10.0);

    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-3.0, 3.0);

    pass.filter(*cloud_filtered);
    //   std::cerr << "Cloud after filtering: " << std::endl;
    //   for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    //     std::cerr << "    " << cloud_filtered->points[i].x << " "
    //                         << cloud_filtered->points[i].y << " "
    //                         << cloud_filtered->points[i].z << std::endl;

    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(cloud);

    // while (!viewer.wasStopped())
    // {
    // }

    pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud_filtered);

    return (0);
}