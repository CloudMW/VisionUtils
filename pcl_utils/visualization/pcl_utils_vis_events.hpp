//
// Created by mfy on 2025/12/31.
//

#ifndef VISIONUTILS_PCL_UTILS_VIS_EVENT_HPP
#define VISIONUTILS_PCL_UTILS_VIS_EVENT_HPP
#include <spdlog/spdlog.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_utils/common/pcl_utils_cloud_safety.hpp>
pcl::visualization::PCLVisualizer::Ptr g_viewer;

namespace pc_utils {
    namespace vis_events {
        template<typename PointT>
        struct ROIData {
            typename pcl::PointCloud<PointT>::ConstPtr input_cloud;
            typename pcl::PointCloud<PointT>::Ptr roi_cloud;
            PointT min_point;
            PointT max_point;
            bool is_success = false;
        };

        inline void keyboardCB(const pcl::visualization::KeyboardEvent &e, void *) {
            if (e.getKeySym() == "x" && e.keyDown()) {
                std::cout << "\n[INFO] 'x' key pressed: Entering area selection mode." << std::endl;
                std::cout << "[HINT] Click and drag to select an area in the viewer." << std::endl;
            }
        }

        template<typename PointT>
        void areaPickingEventOccurred(
            const pcl::visualization::AreaPickingEvent &event,
            void *args) {
            auto *roi_data = static_cast<ROIData<PointT> *>(args);
            const auto names = event.getCloudNames();
            pcl::Indices indices;

            // 获取所有选中的点索引
            event.getPointsIndices(indices);
            // 根据点云名称获取索引（多云场景）
            // const auto names = event.getCloudNames();
            // for (const std::string &name: names) {
            //     indices = event.getPointsIndices(name);
            //
            //     PCL_INFO("Picked %d points from %s \n", indices.size(), name.c_str());
            // }
            auto *cloud_ptr = static_cast<typename pcl::PointCloud<PointT>::ConstPtr *>(args);
            std::cout << "\n[DEBUG] Area picking callback triggered" << std::endl;
            std::cout << "[DEBUG] Indices returned: " << indices.size() << std::endl;
            std::cout << "[DEBUG] Input cloud size: " << (cloud_ptr ? (*cloud_ptr)->size() : 0) << std::endl;

            if (indices.empty()) {
                std::cout << "[WARNING] No points selected in the area!" << std::endl;
                std::cout << "[HINT] Make sure you pressed 'x' key first to enter selection mode" << std::endl;
                return;
            }

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            inliers->indices = indices;
            auto cloud_roi = std::make_shared<pcl::PointCloud<PointT> >();
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(*cloud_ptr);
            extract.setIndices(inliers);
            extract.setNegative(false); // false = 提取 ROI
            cloud_roi->clear();
            extract.filter(*cloud_roi);

            std::cout << "Selected points: " << cloud_roi->size() << std::endl;

            // 计算AABB包围盒
            if (cloud_roi->empty())
                return;

            PointT min_pt, max_pt;
            min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
            max_pt.x = max_pt.y = max_pt.z = std::numeric_limits<float>::lowest();

            for (const auto &pt: cloud_roi->points) {
                min_pt.x = std::min(min_pt.x, pt.x);
                min_pt.y = std::min(min_pt.y, pt.y);
                min_pt.z = std::min(min_pt.z, pt.z);
                max_pt.x = std::max(max_pt.x, pt.x);
                max_pt.y = std::max(max_pt.y, pt.y);
                max_pt.z = std::max(max_pt.z, pt.z);
            }

            std::cout << "========== ROI AABB ==========" << std::endl;
            std::cout << "Min Point: [" << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << "]" << std::endl;
            std::cout << "Max Point: [" << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << "]" << std::endl;
            std::cout << "Size: [" << (max_pt.x - min_pt.x) << ", "
                    << (max_pt.y - min_pt.y) << ", "
                    << (max_pt.z - min_pt.z) << "]" << std::endl;
            std::cout << "==============================" << std::endl;

            // 在可视化中显示ROI
            roi_data->min_point = min_pt;
            roi_data->max_point = max_pt;
            roi_data->roi_cloud = cloud_roi;
            roi_data->is_success = true;

            if (g_viewer) {
                // 移除之前的ROI点云（如果存在）
                g_viewer->removePointCloud("cloud_roi");
                g_viewer->removeShape("roi_bbox");

                // 添加ROI点云，用红色显示
                pcl::visualization::PointCloudColorHandlerCustom<PointT> red_color(cloud_roi, 255, 0, 0);
                g_viewer->addPointCloud(cloud_roi, red_color, "cloud_roi");
                g_viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_roi");

                // 添加AABB包围盒
                g_viewer->addCube(min_pt.x, max_pt.x,
                                  min_pt.y, max_pt.y,
                                  min_pt.z, max_pt.z,
                                  0.0, 1.0, 0.0, // 绿色边框
                                  "roi_bbox");
                g_viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                    pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                    "roi_bbox");
                g_viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "roi_bbox");
            }
        }


        template<typename PointT>
        void areaPickEvent(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                           pcl::PointXYZ &min_point, pcl::PointXYZ &max_point) {
            // 检查点云安全性
            auto cloud_status = pcl_utils::cloud_safety::check<PointT>(cloud, 0);
            if (cloud_status != pcl_utils::cloud_safety::CloudStatus::OK) {
                pcl_utils::cloud_safety::printError("areaPickEvent", cloud_status);
                return;
            }


            // 将加载的点云赋值给全局变量，供回调函数使用

            pcl::visualization::PCLVisualizer::Ptr viewer(
                new pcl::visualization::PCLVisualizer("ROI Picker"));

            // 将viewer赋值给全局变量，供回调函数使用
            g_viewer = viewer;

            viewer->addPointCloud(cloud, "cloud_1");
            viewer->setBackgroundColor(0, 0, 0);
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_1");

            // 注册键盘回调和区域选择回调

            ROIData<PointT> roi_data;
            roi_data.input_cloud = cloud;
            roi_data.is_success = false;
            viewer->registerKeyboardCallback(keyboardCB);
            viewer->registerAreaPickingCallback(areaPickingEventOccurred<PointT>, (void *) &roi_data);

            std::cout << "========================================" << std::endl;
            std::cout << "    PCL ROI 选择工具" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "📖 使用说明：" << std::endl;
            std::cout << "  1️⃣  按 'x' 键进入区域选择模式" << std::endl;
            std::cout << "  2️⃣  按住鼠标左键拖动选择 ROI" << std::endl;
            std::cout << "  3️⃣  释放鼠标查看结果" << std::endl;
            std::cout << "  ❌ 按 'q' 键退出程序" << std::endl;
            std::cout << "========================================" << std::endl;
            std::cout << "💡 提示：如果选不中点，请确保先按 'x' 键！" << std::endl;
            std::cout << "========================================\n" << std::endl;

            while (!viewer->wasStopped()) {
                viewer->spinOnce(100);
            }
            if (!roi_data.is_success) {
                spdlog::info("[ERROR] ROI selection failed or was not completed");
                min_point = pcl::PointXYZ(0,0,0);
                max_point = pcl::PointXYZ(0,0,0);
                return;
            } else {
                min_point = roi_data.min_point;
                max_point = roi_data.max_point;
                return;
            }
        }
    }
}

#endif //VISIONUTILS_PCL_UTILS_VIS_EVENT_HPP
