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
#include <vtkParametricTorus.h>
#include <vtkParametricFunctionSource.h>
#include <map>
#include <set>
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
         *   @code
        *   PCLVisContainer vis_container;
        *   vis_container.addPointCloud("cloud1", cloud1, 3.0);
        *   vis_container.addPointCloud("cloud2", cloud2);
        *   vis_container.show();
        *@endcode
         *
         *
         *
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
            std::set<std::string> shape_ids_; // 保存几何形状 ID

            // 几何形状持久化存储
            struct ShapeData {
                std::string type; // "sphere", "cube", "cylinder", "plane", "torus", "arrow", "text3d", "text"
                std::any data; // 存储形状的参数
            };

            std::map<std::string, ShapeData> shapes_; // 保存形状数据用于持久化

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

                // 重新添加所有几何形状
                for (const auto &[shape_id, shape_data]: shapes_) {
                    if (shape_data.type == "sphere") {
                        auto params = std::any_cast<std::tuple<pcl::PointXYZ, double, int, int, int> >(shape_data.data);
                        viewer_->addSphere(std::get<0>(params), std::get<1>(params),
                                           std::get<2>(params) / 255.0, std::get<3>(params) / 255.0,
                                           std::get<4>(params) / 255.0, shape_id);
                    } else if (shape_data.type == "cube") {
                        auto params = std::any_cast<std::tuple<double, double, double, double, double, double, int, int,
                            int> >(shape_data.data);
                        viewer_->addCube(std::get<0>(params), std::get<1>(params), std::get<2>(params),
                                         std::get<3>(params), std::get<4>(params), std::get<5>(params),
                                         std::get<6>(params) / 255.0, std::get<7>(params) / 255.0,
                                         std::get<8>(params) / 255.0, shape_id);
                    } else if (shape_data.type == "cylinder") {
                        auto params = std::any_cast<std::tuple<pcl::ModelCoefficients, int, int,
                            int> >(shape_data.data);
                        viewer_->addCylinder(std::get<0>(params), shape_id);
                        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                             std::get<1>(params) / 255.0, std::get<2>(params) / 255.0,
                                                             std::get<3>(params) / 255.0, shape_id);
                    } else if (shape_data.type == "plane") {
                        auto params = std::any_cast<std::tuple<pcl::ModelCoefficients, int, int,
                            int> >(shape_data.data);
                        viewer_->addPlane(std::get<0>(params), shape_id);
                        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                             std::get<1>(params) / 255.0, std::get<2>(params) / 255.0,
                                                             std::get<3>(params) / 255.0, shape_id);
                    } else if (shape_data.type == "torus") {
                        auto params = std::any_cast<std::tuple<double, double, int, int, int> >(shape_data.data);
                        auto torus = vtkSmartPointer<vtkParametricTorus>::New();
                        torus->SetRingRadius(std::get<0>(params));
                        torus->SetCrossSectionRadius(std::get<1>(params));
                        auto torusSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
                        torusSource->SetParametricFunction(torus);
                        torusSource->Update();
                        viewer_->addModelFromPolyData(torusSource->GetOutput(), shape_id);
                        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                             std::get<2>(params) / 255.0, std::get<3>(params) / 255.0,
                                                             std::get<4>(params) / 255.0, shape_id);
                    } else if (shape_data.type == "arrow") {
                        auto params = std::any_cast<std::tuple<pcl::PointXYZ, pcl::PointXYZ, int, int, int, bool> >(
                            shape_data.data);
                        viewer_->addArrow(std::get<1>(params), std::get<0>(params),
                                          std::get<2>(params) / 255.0, std::get<3>(params) / 255.0,
                                          std::get<4>(params) / 255.0,
                                          std::get<5>(params), shape_id);
                    } else if (shape_data.type == "text3d") {
                        auto params = std::any_cast<std::tuple<std::string, pcl::PointXYZ, double, double, double,
                            double> >(shape_data.data);
                        viewer_->addText3D(std::get<0>(params), std::get<1>(params), std::get<2>(params),
                                           std::get<3>(params), std::get<4>(params), std::get<5>(params), shape_id);
                    } else if (shape_data.type == "text") {
                        auto params = std::any_cast<std::tuple<std::string, int, int, int, double, double, double> >(
                            shape_data.data);
                        viewer_->addText(std::get<0>(params), std::get<1>(params), std::get<2>(params),
                                         std::get<3>(params),
                                         std::get<4>(params), std::get<5>(params), std::get<6>(params), shape_id);
                    }
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

            // ==================== 几何形状管理方法 ====================

            /**
             * @brief 添加球体
             * @param id 球体标识符
             * @param center 球心坐标
             * @param radius 半径
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             */
            void addSphere(const std::string &id, const pcl::PointXYZ &center, double radius,
                           int r = 255, int g = 0, int b = 0) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addSphere(center, radius, r / 255.0, g / 255.0, b / 255.0, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "sphere";
                shape_data.data = std::make_tuple(center, radius, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加立方体
             * @param id 立方体标识符
             * @param x_min X轴最小值
             * @param x_max X轴最大值
             * @param y_min Y轴最小值
             * @param y_max Y轴最大值
             * @param z_min Z轴最小值
             * @param z_max Z轴最大值
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             */
            void addCube(const std::string &id,
                         double x_min, double x_max,
                         double y_min, double y_max,
                         double z_min, double z_max,
                         int r = 0, int g = 255, int b = 0) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addCube(x_min, x_max, y_min, y_max, z_min, z_max,
                                 r / 255.0, g / 255.0, b / 255.0, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "cube";
                shape_data.data = std::make_tuple(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加圆柱体
             * @param id 圆柱体标识符
             * @param coefficients 圆柱体参数 (point_on_axis.x, point_on_axis.y, point_on_axis.z,
             *                                 axis_direction.x, axis_direction.y, axis_direction.z, radius)
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             */
            void addCylinder(const std::string &id, const pcl::ModelCoefficients &coefficients,
                             int r = 0, int g = 0, int b = 255) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addCylinder(coefficients, id);
                viewer_->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0, b / 255.0, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "cylinder";
                shape_data.data = std::make_tuple(coefficients, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加平面
             * @param id 平面标识符
             * @param coefficients 平面参数 (a, b, c, d) 表示平面方程 ax+by+cz+d=0
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             */
            void addPlane(const std::string &id, const pcl::ModelCoefficients &coefficients,
                          int r = 128, int g = 128, int b = 128) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addPlane(coefficients, id);
                viewer_->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0, b / 255.0, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "plane";
                shape_data.data = std::make_tuple(coefficients, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加圆环（Torus）
             * @param id 圆环标识符
             * @param ring_radius 环的半径
             * @param cross_section_radius 截面半径
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             */
            void addTorus(const std::string &id, double ring_radius = 1.0,
                          double cross_section_radius = 0.2,
                          int r = 255, int g = 128, int b = 0) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }

                // 创建圆环参数化表面
                auto torus = vtkSmartPointer<vtkParametricTorus>::New();
                torus->SetRingRadius(ring_radius);
                torus->SetCrossSectionRadius(cross_section_radius);

                auto torusSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
                torusSource->SetParametricFunction(torus);
                torusSource->Update();

                viewer_->addModelFromPolyData(torusSource->GetOutput(), id);
                viewer_->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, r / 255.0, g / 255.0, b / 255.0, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "torus";
                shape_data.data = std::make_tuple(ring_radius, cross_section_radius, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加箭头
             * @param id 箭头标识符
             * @param pt_start 起点
             * @param pt_end 终点
             * @param r 红色分量 (0-255)
             * @param g 绿色分量 (0-255)
             * @param b 蓝色分量 (0-255)
             * @param display_length 是否显示长度标注
             */
            void addArrow(const std::string &id,
                          const pcl::PointXYZ &pt_start,
                          const pcl::PointXYZ &pt_end,
                          int r = 255, int g = 255, int b = 0,
                          bool display_length = false) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addArrow(pt_end, pt_start, r / 255.0, g / 255.0, b / 255.0, display_length, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "arrow";
                shape_data.data = std::make_tuple(pt_start, pt_end, r, g, b, display_length);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加3D文本
             * @param id 文本标识符
             * @param text 文本内容
             * @param position 3D位置
             * @param text_scale 文本缩放
             * @param r 红色分量 (0-1)
             * @param g 绿色分量 (0-1)
             * @param b 蓝色分量 (0-1)
             */
            void addText3D(const std::string &id,
                           const std::string &text,
                           const pcl::PointXYZ &position,
                           double text_scale = 0.1,
                           double r = 1.0, double g = 1.0, double b = 1.0) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addText3D(text, position, text_scale, r, g, b, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "text3d";
                shape_data.data = std::make_tuple(text, position, text_scale, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 添加2D屏幕文本
             * @param id 文本标识符
             * @param text 文本内容
             * @param x 屏幕X坐标 (像素)
             * @param y 屏幕Y坐标 (像素)
             * @param font_size 字体大小
             * @param r 红色分量 (0-1)
             * @param g 绿色分量 (0-1)
             * @param b 蓝色分量 (0-1)
             */
            void addText(const std::string &id,
                         const std::string &text,
                         int x, int y,
                         int font_size = 14,
                         double r = 1.0, double g = 1.0, double b = 1.0) {
                if (shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 已存在" << std::endl;
                    return;
                }
                viewer_->addText(text, x, y, font_size, r, g, b, id);
                shape_ids_.insert(id);

                // 保存形状数据用于持久化
                ShapeData shape_data;
                shape_data.type = "text";
                shape_data.data = std::make_tuple(text, x, y, font_size, r, g, b);
                shapes_[id] = shape_data;
            }

            /**
             * @brief 删除几何形状
             * @param id 形状标识符
             */
            void removeShape(const std::string &id) {
                if (!shape_ids_.count(id)) {
                    std::cerr << "警告: 形状 ID '" << id << "' 不存在" << std::endl;
                    return;
                }
                viewer_->removeShape(id);
                shape_ids_.erase(id);
                shapes_.erase(id); // 同时删除持久化数据
            }

            /**
             * @brief 删除所有几何形状
             */
            void removeAllShapes() {
                for (const auto &id: shape_ids_) {
                    viewer_->removeShape(id);
                }
                shape_ids_.clear();
                shapes_.clear(); // 同时清除所有持久化数据
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
