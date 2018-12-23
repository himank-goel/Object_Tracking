#include <pcl/point_cloud.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct coordinate
{
    float x, y, z;

    bool operator<(const coordinate &pt) const
    {
        return (x < pt.x) || ((!(pt.x < x)) && (y < pt.y)) || ((!(pt.x < x)) && (!(pt.y < y)) && (z < pt.z));
    }
};

std::set<coordinate> file_to_set(pcl::PointCloud<pcl::PointXYZ>::Ptr file)
{
    std::set<coordinate> points;

    for(size_t i=0; i<file->points.size(); ++i) {
        coordinate *point = new coordinate;
        
        point->x = file->points[i].x;
        point->y = file->points[i].y;
        point->z = file->points[i].z;

        points.insert(*point);
    }

    return points;
}

std::set<coordinate> add_sets(std::set<coordinate> set_1, std::set<coordinate> set_2)
{
    std::set<coordinate>::iterator it;

    for (it = set_2.begin(); it != set_2.end(); ++it)
    {
        coordinate *point = new coordinate;
        
        point->x = (*it).x;
        point->y = (*it).y;
        point->z = (*it).z;
        
        set_1.insert(*point);
    }

    return set_1;

}

pcl::PointCloud<pcl::PointXYZ> get_cloud(std::set<coordinate> coordinates)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = coordinates.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    size_t i = 0;
    std::set<coordinate>::iterator it;

    for (it = coordinates.begin(); it != coordinates.end(); ++it)
    {
        cloud.points[i].x = (*it).x;
        cloud.points[i].y = (*it).y;
        cloud.points[i].z = (*it).z;
        ++i;
    }

    return cloud;
}

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr file_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr file_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    
    std::set<coordinate> file_1_points;
    std::set<coordinate> file_2_points;
    std::set<coordinate> final_points;

    std::stringstream file_name;
    file_name << "../build/back_pcd/back_pcd_000002.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name.str(), *file_1) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file back.pcd \n");
        return (-1);
    }
    
    file_name.str(std::string());
    file_name << "../build/back_pcd/back_pcd_000003.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name.str(), *file_2) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file back.pcd \n");
        return (-1);
    }

    file_1_points = file_to_set(file_1);
    file_2_points = file_to_set(file_2);

    final_points = add_sets(file_1_points, file_2_points);
    final_cloud = get_cloud(final_points);

    file_name.str(std::string());
    file_name << "../build/back_pcd_new/back_compiled.pcd";
    pcl::io::savePCDFileASCII(file_name.str(), final_cloud);
    
  return (0);
}