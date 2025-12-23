//
// Created by mfy on 2025/12/2.
//
#include <pcl/common/common.h>

#include "pcl_utils/visualization/pcl_utils_visualization.hpp"
#include "pcl_utils/io/pcl_utils_io_txt.hpp"
#include "pcl_utils/segmentation/pcl_utils_clusters.hpp"
#include "segmentation/pcl_uitls_sac_segment.hpp"
#include "spdlog/spdlog.h"

typedef  pcl::PointXYZI PointT;
int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    std::string file_2 = std::string(DEMO_PATH) + "/data/2.txt";
    spdlog::info("file_1: {}",file_1);
    spdlog::info("file_2: {}",file_2);
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();


    if (pcl_utils::io::readTXTToPCLXYZI(file_1,cloud_1)&&pcl_utils::io::readTXTToPCLXYZI(file_2,cloud_2)) {

        // clusters
        {
            /*
            {
                //euclideanClustering
                std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
                auto cloudRGB = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

                pcl_utils::segmentation::euclideanClustering<PointT>(cloud_2, clusters, cloudRGB,10 ,10 ,100000);
                spdlog::info("clusters size: {}",clusters.size());
               // pcl_utils::visualization::showPointCloud<pcl::PointXYZRGB>("clusters", cloudRGB);
            }
            */
        }


        //seg
        {
            /*pcl_utils::sac_segmentation::fitAndSegment<PointT>(cloud_1,
                                                               pcl::SACMODEL_PLANE,
                                                               10,
                                                               10000,
                                                               plane_cloud,
                                                               remaining_cloud,
                                                               planeCoefficients
            );*/
            auto container_cloud = std::make_shared<pcl::PointCloud<PointT>>();
            auto container_coefficients = std::make_shared<pcl::ModelCoefficients>();
            pcl_utils::sac_segmentation::fitAndSegment<PointT>(cloud_2,
                                                               pcl::SACMODEL_NORMAL_PLANE,
                                                               20.0,
                                                               1000,
                                                               container_cloud,
                                                               nullptr,
                                                               container_coefficients,
                                                               200.0,
                                                               Eigen::Vector3f(1,0,0),30.0
            );
        }

    }
}