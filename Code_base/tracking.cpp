#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

struct coordinate
{
    float x, y, z;

    bool operator<(const coordinate &pt) const
    {
        return (x < pt.x) || ((!(pt.x < x)) && (y < pt.y)) || ((!(pt.x < x)) && (!(pt.y < y)) && (z < pt.z));
    }
};

std::string addLeadingZero(int k)
{
    std::stringstream ss;
    ss << k;
    std::string final = ss.str();
    int len = final.length();
    for (int i = len; i < 3; i++)
    {
        final.insert(0, "0");
    }
    return final;
}

bool checkVicinity(float value, float boundary)
{
    return (float(boundary - 0.03) <= value && value <= float(boundary + 0.03));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filtering_func(std::string fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr file(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr file_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *file) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return file;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(file);
    vg.setLeafSize(0.08f, 0.08f, 0.08f);
    vg.filter(*file_filtered);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(file_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.96, 12.21);

    pass.filter(*file_filtered);

    pass.setInputCloud(file_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.46, 2.34);

    pass.filter(*file_filtered);

    return file_filtered;
}

std::set<coordinate> findClosePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr prev_human_file, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, float radius, pcl::PointCloud<pcl::PointXYZ>::Ptr current_file_filtered)
{
    std::set<coordinate> human_coordinates;

    for (size_t i = 0; i < prev_human_file->points.size(); i++)
    {

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(prev_human_file->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
            {
                coordinate *current_point = new coordinate;
                current_point->x = current_file_filtered->points[pointIdxRadiusSearch[j]].x;
                current_point->y = current_file_filtered->points[pointIdxRadiusSearch[j]].y;
                current_point->z = current_file_filtered->points[pointIdxRadiusSearch[j]].z;
                human_coordinates.insert(*current_point);
            }
        }
    }
    return human_coordinates;
}

std::set<coordinate> subtractSets(std::set<coordinate> setA, std::set<coordinate> setB)
{
    std::set<coordinate> final_coordinates;

    std::set<coordinate>::iterator itr;
    std::set<coordinate>::iterator it;

    for (itr = setA.begin(); itr != setA.end(); ++itr)
    {
        int flag = 0;
        for (it = setB.begin(); it != setB.end(); it++)
        {
            if ((*it).x == (*itr).x && (*it).y == (*itr).y && (*it).z == (*itr).z)
            {
                flag = 1;
            }
        }
        if (flag != 1)
        {
            final_coordinates.insert(*itr);
        }
    }
    return final_coordinates;
}

std::set<coordinate> backgroundValidation(std::set<coordinate> human_coordinates, pcl::PointCloud<pcl::PointXYZ>::Ptr prev_back_file)
{
    std::set<coordinate>::iterator it;
    std::set<coordinate> exclusive_coordinates;

    for (it = human_coordinates.begin(); it != human_coordinates.end(); ++it)
    {
        for (size_t ptr = 0; ptr < prev_back_file->points.size(); ++ptr)
        {
            if (checkVicinity(prev_back_file->points[ptr].x, (*it).x) && checkVicinity(prev_back_file->points[ptr].y, (*it).y) && checkVicinity(prev_back_file->points[ptr].z, (*it).z))
            {
                coordinate *point = new coordinate;
                point->x = (*it).x;
                point->y = (*it).y;
                point->z = (*it).z;
                exclusive_coordinates.insert(*point);
            }
        }
    }

    return exclusive_coordinates;
}

std::set<coordinate> findBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr current_file_filtered, std::set<coordinate> human_coordinates)
{
    std::set<coordinate> back_coordinates;
    std::set<coordinate> total_coordinates;

    for (size_t m = 0; m < current_file_filtered->points.size(); m++)
    {
        coordinate *point = new coordinate;
        point->x = current_file_filtered->points[m].x;
        point->y = current_file_filtered->points[m].y;
        point->z = current_file_filtered->points[m].z;
        total_coordinates.insert(*point);
    }

    back_coordinates = subtractSets(total_coordinates, human_coordinates);
    return back_coordinates;
}

pcl::PointCloud<pcl::PointXYZ> getCloud(std::set<coordinate> coordinates)
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
    //Variables for maintaining cloud version
    int current_file_number = 261;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_file_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_human_file(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_back_file(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    //To store the human_cloud
    pcl::PointCloud<pcl::PointXYZ> human_cloud;
    std::set<coordinate> human_coordinates;

    //To store the bg_cloud
    pcl::PointCloud<pcl::PointXYZ> back_cloud;
    std::set<coordinate> back_coordinates;

    std::set<coordinate> new_human_coordinates;

    for (int l = 262; l <= 495; l++)
    {
        std::cout << current_file_number << std::endl;

        // Load Current File for inspection
        std::stringstream current_file_name;
        current_file_name << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000" << addLeadingZero(current_file_number) << ".pcd";

        current_file_filtered = filtering_func(current_file_name.str());

        // Load prev human file
        std::stringstream prev_human_file_name;
        prev_human_file_name << "../build/human_pcd/human_pcd_000" << addLeadingZero(current_file_number - 1) << ".pcd";

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(prev_human_file_name.str(), *prev_human_file) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file human.pcd \n");
            return (-1);
        }

        // Load prev back file
        std::stringstream prev_back_file_name;
        prev_back_file_name << "../build/back_pcd/back_pcd_000" << addLeadingZero(current_file_number - 1) << ".pcd";

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(prev_back_file_name.str(), *prev_back_file) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file back.pcd \n");
            return (-1);
        }

        //Kdtree
        kdtree.setInputCloud(current_file_filtered);

        // Aux Variables
        float radius = 0.05;

        // Find initial set of human points
        human_coordinates = findClosePoints(prev_human_file, kdtree, radius, current_file_filtered);

        // Maintain a minimum number of 20 points to represent human by increasing the radius
        while (human_coordinates.size() < prev_human_file->points.size()-10)
        {
            radius += 0.01;
            human_coordinates = findClosePoints(prev_human_file, kdtree, radius, current_file_filtered);
        }

        // Remove all points from humanm that are in vicinity to the previous background file
        std::set<coordinate> exclusive_coordinates;
        // exclusive_coordinates = backgroundValidation(human_coordinates, prev_back_file);
        new_human_coordinates = subtractSets(human_coordinates, exclusive_coordinates);

        // Get human cloud
        human_cloud = getCloud(new_human_coordinates);

        // Get background coordinates and its corresponding cloud
        back_coordinates = findBackground(current_file_filtered, new_human_coordinates);
        back_cloud = getCloud(back_coordinates);

        // Write the human and background to files
        std::stringstream current_human_file_name;
        current_human_file_name << "../build/human_pcd/human_pcd_000" << addLeadingZero(current_file_number) << ".pcd";

        pcl::io::savePCDFileASCII(current_human_file_name.str(), human_cloud);

        std::stringstream current_back_file_name;
        current_back_file_name << "../build/back_pcd/back_pcd_000" << addLeadingZero(current_file_number) << ".pcd";

        pcl::io::savePCDFileASCII(current_back_file_name.str(), back_cloud);

        current_file_number++;
    }
    return (0);
}