//
// Created by mfy on 2025/12/2.
//
#include "pcdl/visualization/pcdl_visualization.h"
#include "pcdl/io/pcdl_io_txt.h"
#include "spdlog/spdlog.h"

int main() {
    std::string file_1 = std::string(VIS_DEMO_PATH)+ "/data/1.txt";
    std::string file_2 = std::string(VIS_DEMO_PATH)+ "/data/1.txt";
    spdlog::info("file_1: {}",file_1);
    spdlog::info("file_2: {}",file_2);
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcdl::io::readTXTToPCLXYZI(file_1,cloud_1)&&pcdl::io::readTXTToPCLXYZI(file_1,cloud_2)) {
        pcdl::visualization::showPointCloud<pcl::PointXYZI>(cloud_1,cloud_2,"cloud_1&cloud_2");
    }
}