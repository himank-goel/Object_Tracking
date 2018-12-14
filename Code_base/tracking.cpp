#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

struct coordinate {
    float x, y, z;

    bool operator <(const coordinate& pt) const {
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
    pass.setFilterLimits(-2.53, 2.34);

    pass.filter(*file_filtered);

    return file_filtered;
}

int main(int argc, char **argv)
{
    //Variables for maintaining cloud version
    int current_file_number = 3;

    for (int l = 4; l <= 50; l++)
    {
        //To store the human_cloud
        pcl::PointCloud<pcl::PointXYZ> human_cloud;
        std::set<coordinate> human_coordinates;

        // Load Current File for inspection
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_file_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        std::stringstream current_file_name;
        current_file_name << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000" << addLeadingZero(current_file_number) << ".pcd";

        current_file_filtered = filtering_func(current_file_name.str());

        // Load prev human file
        pcl::PointCloud<pcl::PointXYZ>::Ptr prev_human_file(new pcl::PointCloud<pcl::PointXYZ>);

        std::stringstream prev_human_file_name;
        prev_human_file_name << "../build/human_pcd/human_pcd_000" << addLeadingZero(current_file_number - 1) << ".pcd";

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(prev_human_file_name.str(), *prev_human_file) == -1) //* load the file
        {
            PCL_ERROR("Couldn't read file human.pcd \n");
            return (-1);
        }

        //Kdtree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(current_file_filtered);

        float radius = 0.05;
        if(prev_human_file->points.size() < 15) {
            radius = 0.08;
        }

        // Aux Variables
        int h_count = 0;

        for (size_t i = 0; i < prev_human_file->points.size(); i++)
        {
            std::cout << i << std::endl;

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

        //To store the bg_cloud
        pcl::PointCloud<pcl::PointXYZ> back_cloud;
        std::set<coordinate> back_coordinates;
        std::set<coordinate> total_coordinates;

        for(size_t m = 0; m<current_file_filtered->points.size(); m++) {
            coordinate *point = new coordinate;
            point->x = current_file_filtered->points[m].x;
            point->y = current_file_filtered->points[m].y;
            point->z = current_file_filtered->points[m].z;
            total_coordinates.insert(*point);
        }

        std::set<coordinate>:: iterator itr;
        std::set<coordinate>:: iterator it;

        for(itr = total_coordinates.begin(); itr!=total_coordinates.end(); ++itr) {
            int flag = 0;
            for(it = human_coordinates.begin(); it!=human_coordinates.end(); it++) {
                if((*it).x == (*itr).x && (*it).y == (*itr).y && (*it).z == (*itr).z) {
                    flag = 1;
                }
                if(flag != 1) {
                    back_coordinates.insert(*itr);
                }
            }
        }

        human_cloud.width = human_coordinates.size();
        human_cloud.height = 1;
        human_cloud.is_dense = false;
        human_cloud.points.resize(human_cloud.width * human_cloud.height);

        size_t i = 0;
        for( it = human_coordinates.begin(); it!=human_coordinates.end(); ++it){
            human_cloud.points[i].x = (*it).x;
            human_cloud.points[i].y = (*it).y;
            human_cloud.points[i].z = (*it).z;
            ++i;
        }

        back_cloud.width = back_coordinates.size();
        back_cloud.height = 1;
        back_cloud.is_dense = false;
        back_cloud.points.resize(back_cloud.width * back_cloud.height);

        i = 0;
        for( it = back_coordinates.begin(); it!=back_coordinates.end(); ++it){
            back_cloud.points[i].x = (*it).x;
            back_cloud.points[i].y = (*it).y;
            back_cloud.points[i].z = (*it).z;
            ++i;
        }

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