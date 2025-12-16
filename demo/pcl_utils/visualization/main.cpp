//
// Created by mfy on 2025/12/2.
//
#include <pcl/common/common.h>

#include "pcl_utils/visualization/pcl_utils_visualization.hpp"
#include "pcl_utils/io/pcl_utils_io_txt.hpp"
#include "spdlog/spdlog.h"
#include <pcl/io/pcd_io.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

#include "pcl_utils/visualization/pcl_utils_vis_container.hpp"
#include "pcl_utils/filters/filters.hpp"
#include <pcl/common/transforms.h>

void pcl_utils_vis_container(
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud, pcl::PointCloud<pcl
        ::PointXYZRGB>::Ptr xyzrgb_cloud
);

#define  TEST_2  1

int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    std::string file_2 = std::string(DEMO_PATH) + "/data/2.txt";
    std::string file_3 = std::string(DEMO_PATH) + "/data/3.txt";
    std::string file_pcd_1 = std::string(DEMO_PATH) + "/data/1.pcd";
    spdlog::info("file_1: {}", file_1);
    spdlog::info("file_2: {}", file_2);

    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
#if TEST_1 >0
    {
        if (pcl_utils::io::readTXTToPCLXYZI(file_1, cloud_1) && pcl_utils::io::readTXTToPCLXYZI(file_2, cloud_2)) {
            // showPointCloud
            {
                // pcdl::visualization::showPointCloud<pcl::PointXYZI>(cloud_1,cloud_2,"cloud_1&cloud_2");
            }


            //
            {
                //variadic templates showPointCloud
                pcl_utils::visualization::showPointCloud<pcl::PointXYZI>("variadic templates showPointCloud", cloud_1,
                                                                         cloud_2);
            }
            //showAABB
            {
                pcl::PointXYZI min_pt, max_pt;
                pcl::getMinMax3D(*cloud_1, min_pt, max_pt);
                pcl_utils::visualization::showAABB<pcl::PointXYZI>(cloud_1, min_pt, max_pt, "cloud_1&cloud_2");
            }
        }
    }
#endif

#if TEST_2>0

    {
        auto xyz_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        auto xyzi_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
        auto xyzrgb_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
        if (pcl_utils::io::readTXTToPCLXYZ(file_1, xyz_cloud) && pcl_utils::io::readTXTToPCLXYZI(file_3, xyzi_cloud)) {
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_pcd_1, *xyzrgb_cloud);
            pcl_utils_vis_container(xyz_cloud, xyzi_cloud, xyzrgb_cloud);
        }
    }
}
#endif

void pcl_utils_vis_container(
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud, pcl::PointCloud<pcl
        ::PointXYZRGB>::Ptr xyzrgb_cloud
) {
    pcl_utils::visualization::PCLVisContainer container;
    // 添加不同类型的点云
    container.addPointCloud("xyz_cloud", xyz_cloud);
    container.addPointCloud("xyzi_cloud", xyzi_cloud);
    container.addPointCloud("xyzrgb_cloud", xyzrgb_cloud);
    container.show();
    // 切换可见性
    container.toggleVisibility("xyz_cloud");
    container.show();
    // 删除点云
    container.toggleVisibility("xyz_cloud");
    container.removePointCloud("xyzi_cloud");
    container.show();



    // 添加球体
    pcl::PointXYZ sphere_center(0.0f, 0.0f, 0.0f);
    container.addSphere("sphere1", sphere_center, 1000, 255, 0, 0);
    std::cout << "✓ 添加了红色球体\n";
    container.show();
    //移除球体
    container.removeShape("sphere1");
    container.show();
}
