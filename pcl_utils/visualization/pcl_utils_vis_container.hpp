//
// Created by mfy on 2025/12/15.
//

#ifndef VISIONUTILS_PCL_UTILS_VIS_CONTAINER_HPP
#define VISIONUTILS_PCL_UTILS_VIS_CONTAINER_HPP
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vtkTextActor.h>
#include <vtkSmartPointer.h>
#include <map>
#include <string>
#include <memory>
#include <any>
#include <vector>
#include <iostream>
#include <array>

namespace pcl_utils {
    namespace visualization {
        /**
         * @brief 可视化容器类 - 用于存储和管理多个点云的可视化
         *
         * 使用示例:
         *   PCLVisContainer vis_container;
         *   vis_container.addPointCloud("cloud1", cloud1, 3.0);
         *   vis_container.addPointCloud("cloud2", cloud2);
         *   vis_container.showAll();
         */
        class PCLVisContainer {
        private:
            // 色表定义 (RGB值: 0-255)
            static const std::vector<std::array<int, 3> > color_table_;
            int current_color_index_ = 0; // 当前使用的颜色索引
            pcl::visualization::PCLVisualizer::Ptr viewer_;
            std::map<std::string, std::any> point_clouds_; // 存储不同类型的点云
            std::map<std::string, bool> visibility_; // 控制点云可见性
            std::map<std::string, double> point_sizes_; // 存储点云大小
            std::map<std::string, std::any> color_handlers_; // 存储颜色处理器
            bool is_initialized_ = false; // 标记是否已初始化
            std::vector<vtkSmartPointer<vtkTextActor> > text_actors_; // 保存文本 actors

            /**
             * @brief 初始化可视化界面
             */
            void initialize() {
                if (is_initialized_) return;

                // 清除旧的文本 actors
                if (!text_actors_.empty()) {
                    auto renderer = viewer_->getRendererCollection()->GetFirstRenderer();
                    for (auto &actor: text_actors_) {
                        renderer->RemoveActor2D(actor);
                    }
                    text_actors_.clear();
                }

                // 创建文本 actors
                auto cloud_size = point_clouds_.size();
                text_actors_.resize(cloud_size);

                int idx = 0;
                for (const auto &[cloud_id, _]: point_clouds_) {
                    text_actors_[idx] = vtkSmartPointer<vtkTextActor>::New();
                    std::string checkbox = visibility_[cloud_id] ? "[X]" : "[ ]";

                    // 显示快捷键提示
                    std::string shortcut;
                    if (idx < 9) {
                        shortcut = std::to_string(idx + 1);
                    } else if (idx < 35) {
                        shortcut = std::string(1, 'a' + (idx - 9));
                    } else {
                        shortcut = "-";
                    }

                    text_actors_[idx]->SetInput((checkbox + " [" + shortcut + "] " + cloud_id).c_str());
                    text_actors_[idx]->GetTextProperty()->SetFontSize(16);
                    text_actors_[idx]->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
                    text_actors_[idx]->SetPosition(10, 100 - idx * 30);
                    viewer_->getRendererCollection()->GetFirstRenderer()->AddActor2D(text_actors_[idx]);
                    idx++;
                }

                // 创建点云 ID 列表
                std::vector<std::string> cloud_ids;
                for (const auto &[cloud_id, _]: point_clouds_) {
                    cloud_ids.push_back(cloud_id);
                }

                // 注册键盘回调
                viewer_->registerKeyboardCallback(
                    [this, cloud_ids](const pcl::visualization::KeyboardEvent &event) {
                        if (!event.keyDown()) return;
                        char key = event.getKeyCode();

                        int idx = -1;

                        // 数字键 1-9
                        if (key >= '1' && key <= '9') {
                            idx = key - '1';
                        }
                        // 字母键 a-z
                        else if (key >= 'a' && key <= 'z') {
                            idx = 9 + (key - 'a');
                        } else if (key >= 'A' && key <= 'Z') {
                            idx = 9 + (key - 'A');
                        }

                        if (idx < 0 || idx >= cloud_ids.size()) return;

                        const std::string &cloud_id = cloud_ids[idx];
                        toggleVisibility(cloud_id);

                        // 更新文本显示
                        std::string checkbox = visibility_[cloud_id] ? "[X]" : "[ ]";
                        std::string shortcut;
                        if (idx < 9) {
                            shortcut = std::to_string(idx + 1);
                        } else if (idx < 35) {
                            shortcut = std::string(1, 'a' + (idx - 9));
                        } else {
                            shortcut = "-";
                        }
                        text_actors_[idx]->SetInput((checkbox + " [" + shortcut + "] " + cloud_id).c_str());

                        std::cout << "点云 " << cloud_id << (visibility_[cloud_id] ? " 显示" : " 隐藏") << std::endl;
                    });

                is_initialized_ = true;
            }

            /**
             * @brief 重置可视化窗口
             */
            void reset() {
                // 创建新的 viewer
                viewer_ = pcl::visualization::PCLVisualizer::Ptr(
                    new pcl::visualization::PCLVisualizer("PCLVisContainer"));
                viewer_->setBackgroundColor(0.0, 0.0, 0.0);
                viewer_->addCoordinateSystem(1.0);
                viewer_->initCameraParameters();
                viewer_->registerKeyboardCallback(&PCLVisContainer::keyboardCallback, *this);

                // 重新添加所有可见的点云
                for (const auto &[cloud_id, _]: point_clouds_) {
                    if (!visibility_[cloud_id]) continue;

                    double point_size = point_sizes_[cloud_id];

                    if (point_clouds_[cloud_id].type() == typeid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(point_clouds_[cloud_id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> > >(
                            color_handlers_[cloud_id]);
                        viewer_->addPointCloud(cloud, *color_handler, cloud_id);
                    } else if (point_clouds_[cloud_id].type() == typeid(pcl::PointCloud<pcl::PointXYZI>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>(point_clouds_[cloud_id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> > >(
                            color_handlers_[cloud_id]);
                        viewer_->addPointCloud(cloud, *color_handler, cloud_id);
                    } else if (point_clouds_[cloud_id].type() == typeid(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<
                            pcl::PointXYZRGB>::ConstPtr>(point_clouds_[cloud_id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > >(
                            color_handlers_[cloud_id]);
                        viewer_->addPointCloud(cloud, *color_handler, cloud_id);
                    }

                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_id);
                }

                is_initialized_ = false;
            }

        public:
            PCLVisContainer()
                : viewer_(new pcl::visualization::PCLVisualizer("PCLVisContainer")) {
                viewer_->setBackgroundColor(0.0, 0.0, 0.0);
                viewer_->addCoordinateSystem(1.0);
                viewer_->initCameraParameters();
                viewer_->registerKeyboardCallback(&PCLVisContainer::keyboardCallback, *this);
            }

            /**
       * @brief 添加点云到容器 (支持 PointXYZ) - 使用色表中的颜色
       * @param id 点云标识符
       * @param cloud 点云数据
       * @param point_size 点的大小,默认为 2.0
       */
            void addPointCloud(const std::string &id,
                               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                               double point_size = 2.0) {
                point_clouds_[id] = cloud;
                visibility_[id] = true;
                point_sizes_[id] = point_size;

                // 从色表中获取颜色
                const auto &color = color_table_[current_color_index_ % color_table_.size()];
                int r = color[0];
                int g = color[1];
                int b = color[2];
                current_color_index_++; // 移动到下一个颜色

                // 创建并保存颜色处理器 (使用固定颜色)
                auto color_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> >(
                    cloud, r, g, b);
                color_handlers_[id] = color_handler;

                viewer_->addPointCloud(cloud, *color_handler, id);
                viewer_->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
            }

            /**
             * @brief 添加点云到容器 (支持 PointXYZI) - 根据强度值上色
             * @param id 点云标识符
             * @param cloud 点云数据
             * @param point_size 点的大小，默认为 2.0
             */
            void addPointCloud(const std::string &id,
                               const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud,
                               double point_size = 2.0) {
                point_clouds_[id] = cloud;
                visibility_[id] = true;
                point_sizes_[id] = point_size;

                // 创建并保存颜色处理器
                auto intensity_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<
                    pcl::PointXYZI> >(
                    cloud, "intensity");
                color_handlers_[id] = intensity_handler;

                viewer_->addPointCloud(cloud, *intensity_handler, id);
                viewer_->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
            }

            /**
             * @brief 添加点云到容器 (支持 PointXYZRGB) - 使用自身颜色
             * @param id 点云标识符
             * @param cloud 点云数据
             * @param point_size 点的大小，默认为 2.0
             */
            void addPointCloud(const std::string &id,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                               double point_size = 2.0) {
                point_clouds_[id] = cloud;
                visibility_[id] = true;
                point_sizes_[id] = point_size;

                // 创建并保存颜色处理器
                auto rgb_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerRGBField<
                    pcl::PointXYZRGB> >(cloud);
                color_handlers_[id] = rgb_handler;

                viewer_->addPointCloud(cloud, *rgb_handler, id);
                viewer_->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
            }

            /**
             * @brief 删除点云
             */
            void removePointCloud(const std::string &id) {
                viewer_->removePointCloud(id);
                point_clouds_.erase(id);
                visibility_.erase(id);
                point_sizes_.erase(id);
                color_handlers_.erase(id);
            }

            /**
             * @brief 切换点云可见性
             */
            void toggleVisibility(const std::string &id) {
                if (visibility_.find(id) == visibility_.end()) return;

                visibility_[id] = !visibility_[id];

                if (visibility_[id]) {
                    double point_size = point_sizes_[id];

                    // 使用保存的颜色处理器
                    if (point_clouds_[id].type() == typeid(pcl::PointCloud<pcl::PointXYZ>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>(point_clouds_[id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> > >(color_handlers_[id]);
                        viewer_->addPointCloud(cloud, *color_handler, id);
                    } else if (point_clouds_[id].type() == typeid(pcl::PointCloud<pcl::PointXYZI>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZI>::ConstPtr>(point_clouds_[id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> > >(
                            color_handlers_[id]);
                        viewer_->addPointCloud(cloud, *color_handler, id);
                    } else if (point_clouds_[id].type() == typeid(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr)) {
                        auto cloud = std::any_cast<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr>(point_clouds_[id]);
                        auto color_handler = std::any_cast<std::shared_ptr<
                            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> > >(
                            color_handlers_[id]);
                        viewer_->addPointCloud(cloud, *color_handler, id);
                    }

                    viewer_->setPointCloudRenderingProperties(
                        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
                } else {
                    viewer_->removePointCloud(id);
                }
            }


            /**
 * @brief 显示可视化窗口（可重复调用）
 */
            void show() {
                // 如果窗口已关闭，重新创建
                if (viewer_->wasStopped()) {
                    reset();
                }

                // 初始化界面
                initialize();

                // 主循环
                while (!viewer_->wasStopped()) {
                    viewer_->spinOnce(100);
                }
            }

            /**
             * @brief 获取可视化器指针
             */
            pcl::visualization::PCLVisualizer::Ptr getViewer() {
                return viewer_;
            }

            /**
             * @brief 键盘回调函数
             */
            void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
                if (event.getKeySym() == "h" && event.keyDown()) {
                    std::cout << "\n=== PCLVisContainer Help ===\n";
                    std::cout << "Available clouds:\n";
                    for (const auto &pair: visibility_) {
                        std::cout << "  " << pair.first << " ["
                                << (pair.second ? "visible" : "hidden")
                                << ", size: " << point_sizes_[pair.first] << "]\n";
                    }
                    std::cout << "Usage: Press 1-9 to toggle cloud visibility\n";
                }
            }
        };

        // 色表定义 - 包含20种区分度高的颜色
        const std::vector<std::array<int, 3> > PCLVisContainer::color_table_ = {
            {255, 255, 255}, //白色
            {255, 0, 0}, // 红色
            {0, 255, 0}, // 绿色
            {0, 0, 255}, // 蓝色
            {255, 255, 0}, // 黄色
            {255, 0, 255}, // 品红
            {0, 255, 255}, // 青色
            {255, 128, 0}, // 橙色
            {128, 0, 255}, // 紫色
            {0, 255, 128}, // 春绿色
            {255, 0, 128}, // 玫瑰红
            {128, 255, 0}, // 黄绿色
            {0, 128, 255}, // 天蓝色
            {255, 128, 128}, // 浅红色
            {128, 255, 128}, // 浅绿色
            {128, 128, 255}, // 浅蓝色
            {255, 255, 128}, // 浅黄色
            {255, 128, 255}, // 浅品红
            {128, 255, 255}, // 浅青色
            {192, 64, 0}, // 棕色
            {64, 192, 0} // 橄榄绿
        };
    }
}

#endif //VISIONUTILS_PCL_UTILS_VIS_CONTAINER_HPP
