//
// Created by mfy on 2025/12/30.
//

#include <io/pcl_utils_io_txt.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <limits>
#include <mutex>
#include "pcl_utils/visualization/pcl_utils_vis_events.hpp"
#include "visualization/pcl_utils_visualization.hpp"
using namespace pcl_utils;
int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    if (pcl_utils::io::readTXTToPCLXYZ(file_1, cloud_1)) {
        pcl::PointXYZ min_point,max_point;
        pc_utils::vis_events::areaPickEvent<pcl::PointXYZ>(cloud_1,min_point,max_point);
        spdlog::info("ROI Min Point: [{}, {}, {}]", min_point.x, min_point.y, min_point.z);
        spdlog::info("ROI Max Point: [{}, {}, {}]", max_point.x, max_point.y, max_point.z);
        pcl_utils::visualization::showAABB<pcl::PointXYZ>(cloud_1, min_point, max_point);
    }
    return 0;
}
