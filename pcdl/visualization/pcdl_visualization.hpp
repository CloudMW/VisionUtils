//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#define POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

namespace pcdl
{
    namespace visualization
    {
        template<typename PointT>
        bool showPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const typename pcl::PointCloud<PointT>::ConstPtr &target_cloud,
                            const std::string &window_name) {
            // 检查点云是否为空
            if (cloud->empty() && target_cloud->empty()) {
                spdlog::error("[showPointCloud] 输入点云为空！");
                return false;
            }
            //11
            // 创建可视化对象
            pcl::visualization::PCLVisualizer viewer(window_name);

            // 设置背景色（黑色，RGB值范围 0-1）
            viewer.setBackgroundColor(0.0, 0.0, 0.0);


            // 设置视角和旋转中心
            viewer.setCameraPosition(1000, 0, 0, 0, 0, 0); // 视点在 (0, 0, 10)，旋转中心为原点 (0, 0, 0)
            viewer.setCameraFieldOfView(0.5); // 可选：调整视角的大小
            // viewer.setSize(800, 600); // 设置窗口大小
            // 添加点云到可视化窗口（参数：点云ID、点云对象）
            // 点云ID用于区分多个点云，相同ID会覆盖
            viewer.addPointCloud<PointT>(cloud, "input_cloud");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> targetColor(target_cloud, 255, 0, 0);
            viewer.addPointCloud<PointT>(target_cloud, targetColor, "target_cloud");

            // 设置点云显示大小（默认1，值越大点越明显）
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "input_cloud");

            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");


            // 添加坐标系（可选，帮助定位，参数：坐标系大小）
            viewer.addCoordinateSystem(1000.0);
            // 启动可视化循环（阻塞，直到关闭窗口）
            spdlog::info("[showPointCloud] 可视化窗口已启动，关闭窗口退出...");
            while (!viewer.wasStopped()) {
                viewer.spinOnce(100); // 每次循环等待100ms，处理事件
            }

            return true;
        }
    }
}



#endif //POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
