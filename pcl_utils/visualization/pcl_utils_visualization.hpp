//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#define POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>
#include <random>
#include <initializer_list>
namespace pcdl
{
    namespace visualization
    {
        /*!
         *
         * @tparam PointT 点云类型
         * @param cloud 参考点云
         * @param target_cloud 目标点云
         * @param window_name 窗口名称
         * @param cloud_size 参考点云大小
         * @param target_cloud_size 目标点云大小
         * @param cam_pos 相机位置
         * @param view_point 相机视点
         * @param cam_up 相机上方向
         * @return
         */
        template <typename PointT>
        bool showPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                            const typename pcl::PointCloud<PointT>::ConstPtr& target_cloud,
                            const std::string& window_name,
                            int cloud_size = 1,
                            int target_cloud_size = 2,
                            const pcl::PointXYZ& cam_pos = pcl::PointXYZ(4000, 3000, 2000),
                            const pcl::PointXYZ& view_point = pcl::PointXYZ(200, 80, 600),
                            const pcl::PointXYZ& cam_up = pcl::PointXYZ(-0.15, -0.17, 0.97)

        )
        {
            // 检查点云是否为空
            if (cloud->empty() || target_cloud->empty())
            {
                //XLOG_ERROR("[showPointCloud] 输入点云为空！");
                return false;
            }
            //11
            // 创建可视化对象
            pcl::visualization::PCLVisualizer viewer(window_name);

            // 设置背景色（黑色，RGB值范围 0-1）
            viewer.setBackgroundColor(0.0, 0.0, 0.0);
            viewer.setCameraPosition(
                cam_pos.x, cam_pos.y, cam_pos.z,
                view_point.x, view_point.y, view_point.z,
                cam_up.x, cam_up.y, cam_up.z); // 视点在 (0, 0, 10)，旋转中心为原点 (0, 0, 0)


            viewer.setCameraFieldOfView(0.5); // 可选：调整视角的大小
            // viewer.setSize(800, 600); // 设置窗口大小
            // 添加点云到可视化窗口（参数：点云ID、点云对象）
            // 点云ID用于区分多个点云，相同ID会覆盖
            viewer.addPointCloud<PointT>(cloud, "input_cloud");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> targetColor(cloud, 255, 0, 0);
            viewer.addPointCloud<PointT>(target_cloud, targetColor, "target_cloud");

            // 设置点云显示大小（默认1，值越大点越明显）
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud_size,
                                                    "input_cloud");
            viewer.addText("Camera Info", 5, 580, 13, 1, 1, 1, "cam_info");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, target_cloud_size,
                                                    "target_cloud");

            viewer.setWindowName(window_name);
            // 添加坐标系（可选，帮助定位，参数：坐标系大小）
            viewer.addCoordinateSystem(1000.0);
            // 启动可视化循环（阻塞，直到关闭窗口）
            spdlog::info("[showPointCloud] show {} ...", window_name);
            bool first_frame = true;
            while (!viewer.wasStopped())
            {
                viewer.spinOnce(100); // 每次循环等待100ms，处理事件
                // ---- 获取相机参数 ----
                // 设置视角和旋转中心
                if (first_frame)
                {
                    viewer.setCameraPosition(
                        cam_pos.x, cam_pos.y, cam_pos.z,
                        view_point.x, view_point.y, view_point.z,
                        cam_up.x, cam_up.y, cam_up.z); // 视点在 (0, 0, 10)，旋转中心为原点 (0, 0, 0)
                    first_frame = false;
                }
                pcl::visualization::Camera cam;
                viewer.getCameraParameters(cam);

                std::stringstream ss;
                ss << "Camera Position: ("
                    << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")\n"
                    << "Focal Point:    ("
                    << cam.focal[0] << ", " << cam.focal[1] << ", " << cam.focal[2] << ")\n"
                    << "View Up:        ("
                    << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")";

                // 更新文本
                viewer.updateText(ss.str(), 5, 5, "cam_info");
            }
            return true;
        }


        /**
         * @brief 单函数实现的可变长参数点云显示器 (C++11/14兼容版本)
         * @param window_name 窗口名称
         * @param args 可变数量的点云指针
         * @return 成功返回true,失败返回false
         *
         * 使用示例:
         * showPointCloud<pcl::PointXYZ>("My Window", cloud1, cloud2, cloud3);
         */
        template <typename PointT, typename... Args>
        bool showPointCloud(const std::string& window_name, Args... args)
        {
            // 创建可视化器
            pcl::visualization::PCLVisualizer::Ptr viewer(
                new pcl::visualization::PCLVisualizer(window_name)
            );

            viewer->setBackgroundColor(0, 0, 0);
            viewer->addCoordinateSystem(1.0);
            viewer->initCameraParameters();

            // 随机数生成器
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);

            // 点云计数器
            int cloud_id = 0;
            int valid_clouds = 0;

            // 使用初始化列表展开参数包
            // 这个技巧利用了逗号表达式和初始化列表的求值顺序
            std::initializer_list<int>{
                ([&](typename pcl::PointCloud<PointT>::Ptr cloud) -> int
                {
                    if (cloud && !cloud->empty())
                    {
                        std::string cloud_name = "cloud_" + std::to_string(cloud_id);

                        // 第一个点云用白色,其他用随机颜色
                        if (cloud_id == 0)
                        {
                            pcl::visualization::PointCloudColorHandlerCustom<PointT>
                                color_handler(cloud, 255, 255, 255);
                            viewer->addPointCloud<PointT>(cloud, color_handler, cloud_name);
                        }
                        else
                        {
                            int r = static_cast<int>(dis(gen) * 255);
                            int g = static_cast<int>(dis(gen) * 255);
                            int b = static_cast<int>(dis(gen) * 255);
                            pcl::visualization::PointCloudColorHandlerCustom<PointT>
                                color_handler(cloud, r, g, b);
                            viewer->addPointCloud<PointT>(cloud, color_handler, cloud_name);
                        }

                        viewer->setPointCloudRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_name
                        );

                        std::cout << "Added cloud_" << cloud_id
                            << " with " << cloud->size() << " points" << std::endl;
                        valid_clouds++;
                    }
                    else
                    {
                        std::cerr << "Warning: Empty or null cloud at position "
                            << cloud_id << std::endl;
                    }
                    cloud_id++;
                    return 0;
                }(args))... // 参数包展开
            };

            if (valid_clouds == 0)
            {
                std::cerr << "Error: No valid point clouds provided" << std::endl;
                return false;
            }

            std::cout << "Total " << valid_clouds << " cloud(s) displayed. Press 'q' to quit."
                << std::endl;

            // 显示窗口
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
            }

            return true;
        }


        /*!
         * @brief 显示点云及其AABB包围盒
         * @tparam PointT
         * @param cloud
         * @param min_pt
         * @param max_pt
         * @param window_name
         * @param cam_pos
         * @param view_point
         * @param cam_up
         */
        template <typename PointT>
        void showAABB(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, PointT min_pt,
                      PointT max_pt,
                      const std::string& window_name = "AABB Viewer",
                      const pcl::PointXYZ& cam_pos = pcl::PointXYZ(4000, 3000, 2000),
                      const pcl::PointXYZ& view_point = pcl::PointXYZ(200, 80, 600),
                      const pcl::PointXYZ& cam_up = pcl::PointXYZ(-0.15, -0.17, 0.97))
        {
            // 创建可视化窗口
            pcl::visualization::PCLVisualizer::Ptr viewer(
                new pcl::visualization::PCLVisualizer(window_name));
            viewer->setBackgroundColor(0, 0, 0);

            // 添加点云
            pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(
                cloud, 255, 255, 255);

            viewer->addPointCloud<PointT>(cloud, cloud_color, "cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

            // 添加AABB包围盒
            viewer->addCube(min_pt.x, max_pt.x,
                            min_pt.y, max_pt.y,
                            min_pt.z, max_pt.z,
                            1.0, 0.0, 0.0, "aabb");
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "aabb");
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "aabb");
            viewer->setWindowName(window_name);
            viewer->setBackgroundColor(0.0, 0.0, 0.0);
            viewer->setCameraPosition(
                cam_pos.x, cam_pos.y, cam_pos.z,
                view_point.x, view_point.y, view_point.z,
                cam_up.x, cam_up.y, cam_up.z); // 视点在 (0, 0, 10)，旋转中心为原点 (0, 0, 0)
            // 显示坐标系
            viewer->addCoordinateSystem(100.0);
            viewer->initCameraParameters();
            spdlog::info("[showCube] show {} ...", window_name);
            // 主循环

            bool first_frame = true;
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                // ---- 获取相机参数 ----
                // 设置视角和旋转中心
                if (first_frame)
                {
                    viewer->setCameraPosition(
                        cam_pos.x, cam_pos.y, cam_pos.z,
                        view_point.x, view_point.y, view_point.z,
                        cam_up.x, cam_up.y, cam_up.z); // 视点在 (0, 0, 10)，旋转中心为原点 (0, 0, 0)
                    first_frame = false;
                }
                pcl::visualization::Camera cam;
                viewer->getCameraParameters(cam);

                std::stringstream ss;
                ss << "Camera Position: ("
                    << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")\n"
                    << "Focal Point:    ("
                    << cam.focal[0] << ", " << cam.focal[1] << ", " << cam.focal[2] << ")\n"
                    << "View Up:        ("
                    << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")";

                // 更新文本
                viewer->updateText(ss.str(), 5, 5, "cam_info");
            }
        }
    }
}


#endif //POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
