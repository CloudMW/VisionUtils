//
// Created by mfy on 2025/12/2.
//
#include <pcl/common/common.h>

#include "pcdl/visualization/pcdl_visualization.hpp"
#include "pcdl/io/pcdl_io_txt.hpp"
#include "pcdl/segmentation/pcdl_clusters.hpp"
#include "spdlog/spdlog.h"

typedef  pcl::PointXYZI PointT;
int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    std::string file_2 = std::string(DEMO_PATH) + "/data/2.txt";
    spdlog::info("file_1: {}",file_1);
    spdlog::info("file_2: {}",file_2);
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();


    if (pcdl::io::readTXTToPCLXYZI(file_1,cloud_1)&&pcdl::io::readTXTToPCLXYZI(file_2,cloud_2)) {

        // clusters
        {
            {
                //euclideanClustering
                std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
                auto cloudRGB = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

                pcdl::segmentation::euclideanClustering<PointT>(cloud_2, clusters, cloudRGB,10 ,10 ,100000);
                spdlog::info("clusters size: {}",clusters.size());
                pcdl::visualization::showPointCloud<pcl::PointXYZRGB>("clusters", cloudRGB);
            }
        }

    }
}