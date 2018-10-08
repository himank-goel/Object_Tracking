#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/visualization/cloud_viewer.h>

float pntX1, pntY1, pntZ1, r;
std::vector<float> x_coordinates, y_coordinates, z_coordinates;

void midPointCircleAlgo()
{
    pcl::PointCloud<pcl::PointXYZ> avg_cloud;
    // plot(x, y);

    std::vector<float> avg_vector;

    avg_vector.push_back(accumulate(x_coordinates.begin(), x_coordinates.end(), 0.0) / x_coordinates.size());
    avg_vector.push_back(accumulate(y_coordinates.begin(), y_coordinates.end(), 0.0) / y_coordinates.size());
    avg_vector.push_back(accumulate(z_coordinates.begin(), z_coordinates.end(), 0.0) / z_coordinates.size());

    for (int i = 0; i < x_coordinates.size(); i++)
    {
        x_coordinates[i] = abs(x_coordinates[i]) - abs(avg_vector[0]);
        y_coordinates[i] = abs(y_coordinates[i]) - abs(avg_vector[1]);
        z_coordinates[i] = abs(z_coordinates[i]) - abs(avg_vector[2]);
    }

    float max_value = 0.0;

    for (int i = 0; i < x_coordinates.size(); i++)
    {
        if (max_value < abs(x_coordinates[i]))
        {
            max_value = abs(x_coordinates[i]);
        }
        if (max_value < abs(y_coordinates[i]))
        {
            max_value = abs(y_coordinates[i]);
        }
    }

    // std::cout << max_value << "Hello" << std::endl;

    r = max_value;
    pntX1 = avg_vector[0];
    pntY1 = avg_vector[1];
    pntZ1 = avg_vector[2];

    float x = 0.0;
    float y = r;
    float decision = 5 / 4 - r;

    avg_vector.push_back(x+pntX1);
    avg_vector.push_back(y+pntY1);
    avg_vector.push_back(pntZ1);

    while (y > x)
    {
        if (decision < 0)
        {
            x = x + 0.1;
            decision += 2 * x + 1;
        }
        else
        {
            y = y - 0.1;
            x = x + 0.1;
            decision += 2 * (x - y) + 1;
        }
        // plot(x, y);
        avg_vector.push_back(x+pntX1);
        avg_vector.push_back(y+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(x, -y);
        avg_vector.push_back(x+pntX1);
        avg_vector.push_back(-y+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(-x, y);
        avg_vector.push_back(-x+pntX1);
        avg_vector.push_back(y+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(-x, -y);
        avg_vector.push_back(-x+pntX1);
        avg_vector.push_back(-y+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(y, x);
        avg_vector.push_back(y+pntX1);
        avg_vector.push_back(x+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(-y, x);
        avg_vector.push_back(-y+pntX1);
        avg_vector.push_back(x+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(y, -x);
        avg_vector.push_back(y+pntX1);
        avg_vector.push_back(-x+pntY1);
        avg_vector.push_back(pntZ1);
        // plot(-y, -x);
        avg_vector.push_back(-y+pntX1);
        avg_vector.push_back(-x+pntY1);
        avg_vector.push_back(pntZ1);
    }

    for(int i=0; i<avg_vector.size(); i++) {
        std::cout<<avg_vector[i]<<"hell"<<std::endl;
    }

    avg_cloud.width = (avg_vector.size()/3);
    avg_cloud.height = 1;
    avg_cloud.is_dense = false;
    avg_cloud.points.resize(avg_cloud.width * avg_cloud.height);

    for (size_t i = 0; i < avg_cloud.points.size(); ++i)
    {
        avg_cloud.points[i].x = avg_vector[(i*3)];
        avg_cloud.points[i].y = avg_vector[(i*3)+1];
        avg_cloud.points[i].z = avg_vector[(i*3)+2];
    }

    pcl::io::savePCDFileASCII("avg_pcd.pcd", avg_cloud);
}

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
    pass.setFilterLimits(5.0, 10.0);

    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, -0.5);

    pass.filter(*cloud_filtered);

    // pass.setInputCloud(cloud_filtered);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(-0.5, 0.5);

    // pass.filter(*cloud_filtered);

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

    for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    {
        x_coordinates.push_back(cloud_filtered->points[i].x);
        y_coordinates.push_back(cloud_filtered->points[i].y);
        z_coordinates.push_back(cloud_filtered->points[i].z);
    }

    midPointCircleAlgo();

    pcl::io::savePCDFileASCII("test_pcd.pcd", *cloud_filtered);

    return (0);
}