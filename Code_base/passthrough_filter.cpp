#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

float pntX1, pntY1, pntZ1, r;
std::vector<float> x_coordinates, y_coordinates, z_coordinates;

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

bool checkVicinity(float value, float boundary) {
    return (float(boundary - 0.05) <= value && value <= float(boundary + 0.05));
}

int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ> background_cloud;
    pcl::PointCloud<pcl::PointXYZ> human_cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr file_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<float> human_x_coordinates, human_y_coordinates, human_z_coordinates;
    std::vector<float> bg_x_coordinates, bg_y_coordinates, bg_z_coordinates;

    std::stringstream test_file;
    test_file << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000020" << ".pcd";
    
    cloud_filtered = filtering_func(test_file.str());

    // for (size_t i = 0; i < cloud_filtered->points.size(); ++i)
    // {
    //     x_coordinates.push_back(cloud_filtered->points[i].x);
    //     y_coordinates.push_back(cloud_filtered->points[i].y);
    //     z_coordinates.push_back(cloud_filtered->points[i].z);
    // }

    int point_count = 0 ;
    int h_count =0 ;
    int b_count = 0;


    // std::cout<<cloud_filtered->points.size()<<std::endl;

    for(size_t i=0; i<cloud_filtered->points.size(); i++) {
        std::cout<<i<<std::endl;
        for(int k=0; k<150; k++) {
            std::stringstream ps;
            ps << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000" << addLeadingZero(k) << ".pcd";
            file_filtered = filtering_func(ps.str());

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(file_filtered);
            
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;

            if(file_filtered->points.size() != 0) {
                if ( kdtree.radiusSearch (cloud_filtered->points[i], 0.03, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                    point_count += pointIdxRadiusSearch.size();
                }    
            }
        }

        std::cout<<point_count<<std::endl;

        if(point_count < 50 && point_count > 10) {
            human_x_coordinates.push_back(cloud_filtered->points[i].x);
            human_y_coordinates.push_back(cloud_filtered->points[i].y);
            human_z_coordinates.push_back(cloud_filtered->points[i].z);
            std::cout<<"human"<<std::endl;
            h_count++;
        } else {
            bg_x_coordinates.push_back(cloud_filtered->points[i].x);
            bg_y_coordinates.push_back(cloud_filtered->points[i].y);
            bg_z_coordinates.push_back(cloud_filtered->points[i].z);
            std::cout<<"bg"<<std::endl;
            b_count++;
        }
        point_count = 0;
    }

    // for(size_t i =0 ; i<cloud_filtered->points.size(); ++i) {
    //     std::cout<<i<<std::endl;
    //     for(int k=0; k<150; k++) {
    //         std::stringstream ps;
    //         ps << "../Pcd_Logs/2018-09-26_19-46-31.939/cloud000" << addLeadingZero(k) << ".pcd";
    //         file_filtered = filtering_func(ps.str());
    //         if(file_filtered->points.size() != 0) {
    //             for(size_t j = 0; j<file_filtered->points.size(); ++j) {
    //                 if(checkVicinity(cloud_filtered->points[i].x, file_filtered->points[j].x) && checkVicinity(cloud_filtered->points[i].y, file_filtered->points[j].y) && checkVicinity(cloud_filtered->points[i].z, file_filtered->points[j].z)) {
    //                     point_count++;
    //                 }
    //             }
    //         }
    //     }
    //     if(point_count < 50) {
    //         human_x_coordinates.push_back(cloud_filtered->points[i].x);
    //         human_y_coordinates.push_back(cloud_filtered->points[i].y);
    //         human_z_coordinates.push_back(cloud_filtered->points[i].z);
    //         std::cout<<"human"<<std::endl;
    //         h_count++;
    //     } else {
    //         bg_x_coordinates.push_back(cloud_filtered->points[i].x);
    //         bg_y_coordinates.push_back(cloud_filtered->points[i].y);
    //         bg_z_coordinates.push_back(cloud_filtered->points[i].z);
    //         std::cout<<"bg"<<std::endl;
    //         b_count++;
    //     }
    //     point_count = 0;
    // }

    human_cloud.width = h_count;
    human_cloud.height = 1;
    human_cloud.is_dense = false;
    human_cloud.points.resize(human_cloud.width * human_cloud.height);

    for (size_t i =0; i<human_cloud.points.size(); ++i) {
        human_cloud.points[i].x = human_x_coordinates[i];
        human_cloud.points[i].y = human_y_coordinates[i];
        human_cloud.points[i].z = human_z_coordinates[i];
    }

    background_cloud.width = b_count;
    background_cloud.height = 1;
    background_cloud.is_dense = false;
    background_cloud.points.resize(background_cloud.width * background_cloud.height);

    for (size_t i =0; i<background_cloud.points.size(); ++i) {
        background_cloud.points[i].x = bg_x_coordinates[i];
        background_cloud.points[i].y = bg_y_coordinates[i];
        background_cloud.points[i].z = bg_z_coordinates[i];
    }

    // midPointCircleAlgo();

    pcl::io::savePCDFileASCII("human_pcd.pcd", human_cloud);
    pcl::io::savePCDFileASCII("back_pcd.pcd", background_cloud);

    return (0);
}