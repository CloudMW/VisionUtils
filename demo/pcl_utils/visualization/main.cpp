//
// Created by mfy on 2025/12/2.
//
#include <pcl/common/common.h>

#include "pcdl/visualization/pcdl_visualization.hpp"
#include "pcdl/io/pcdl_io_txt.hpp"
#include "spdlog/spdlog.h"

int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    std::string file_2 = std::string(DEMO_PATH) + "/data/2.txt";
    spdlog::info("file_1: {}",file_1);
    spdlog::info("file_2: {}",file_2);
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();


    if (pcdl::io::readTXTToPCLXYZI(file_1,cloud_1)&&pcdl::io::readTXTToPCLXYZI(file_2,cloud_2)) {

        // showPointCloud
        {
           // pcdl::visualization::showPointCloud<pcl::PointXYZI>(cloud_1,cloud_2,"cloud_1&cloud_2");
        }


        //
        {
            //variadic templates showPointCloud
           pcdl::visualization::showPointCloud<pcl::PointXYZI>("variadic templates showPointCloud",cloud_1,cloud_2);
        }
        //showAABB
        {
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cloud_1, min_pt, max_pt);
            pcdl::visualization::showAABB<pcl::PointXYZI>(cloud_1,min_pt,max_pt,"cloud_1&cloud_2");
        }

    }
}