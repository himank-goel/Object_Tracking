#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

std::string addLeadingZero(int k) {
    std::stringstream ss;
    ss << k;
    std::string final = ss.str();
    int len = final.length();
    for(int i=len; i<3; i++) {
        final.insert(0, "0");
    }
    return final;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filtering_func(std::string fileName) {

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
    //To store the human_cloud
    pcl::PointCloud<pcl::PointXYZ> human_cloud;
    std::vector<float> human_x_coordinates, human_y_coordinates, human_z_coordinates;
    
    // Load Current File for inspection
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_file_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::stringstream current_file_name;
    current_file_name << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000003" << ".pcd";
    
    current_file_filtered = filtering_func(current_file_name.str());

    // Load prev human file
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_human_file(new pcl::PointCloud<pcl::PointXYZ>);

    std::stringstream prev_human_file_name;
    prev_human_file_name << "../build/human_pcd/human_pcd_000002" << ".pcd";
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(prev_human_file_name.str(), *prev_human_file) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file human.pcd \n");
        return (-1);
    }

    //Kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(current_file_filtered);
        
    // Aux Variables
    int h_count =0 ;

    for(size_t i=0; i<prev_human_file->points.size(); i++) {
        std::cout<<i<<std::endl;
        
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if ( kdtree.radiusSearch (prev_human_file->points[i], 0.03, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
            for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j) {
                human_x_coordinates.push_back(current_file_filtered->points[ pointIdxRadiusSearch[j] ].x);
                human_y_coordinates.push_back(current_file_filtered->points[ pointIdxRadiusSearch[j] ].y);
                human_z_coordinates.push_back(current_file_filtered->points[ pointIdxRadiusSearch[j] ].z);
                h_count++;
            }
        }    
    }

    human_cloud.width = h_count;
    human_cloud.height = 1;
    human_cloud.is_dense = false;
    human_cloud.points.resize(human_cloud.width * human_cloud.height);

    for (size_t i =0; i<human_cloud.points.size(); ++i) {
        human_cloud.points[i].x = human_x_coordinates[i];
        human_cloud.points[i].y = human_y_coordinates[i];
        human_cloud.points[i].z = human_z_coordinates[i];
    }

    pcl::io::savePCDFileASCII("../build/human_pcd/human_pcd_000003.pcd", human_cloud);

    return (0);
}