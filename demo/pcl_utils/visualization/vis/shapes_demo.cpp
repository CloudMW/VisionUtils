/**
 * @file shapes_demo.cpp
 * @brief PCLVisContainer 几何形状演示示例
 *
 * 演示如何使用 PCLVisContainer 添加和删除各种几何形状：
 * - 球体 (Sphere)
 * - 立方体 (Cube)
 * - 圆柱体 (Cylinder)
 * - 平面 (Plane)
 * - 圆环 (Torus)
 * - 箭头 (Arrow)
 * - 3D文本 (Text3D)
 * - 2D屏幕文本 (Text)
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include "pcl_utils/visualization/pcl_utils_vis_container.hpp"

int main() {
    using namespace pcl_utils::visualization;

    // 创建可视化容器
    PCLVisContainer container;

    // ==================== 添加几何形状 ====================

    // 1. 添加球体 - 在原点，半径0.5，红色
    pcl::PointXYZ sphere_center(0.0f, 0.0f, 0.0f);
    container.addSphere("sphere1", sphere_center, 0.5, 255, 0, 0);

    // 2. 添加立方体 - 绿色
    //container.addCube("cube1", -2.0, -1.0, -0.5, 0.5, -0.5, 0.5, 0, 255, 0);

    // 3. 添加圆柱体 - 蓝色
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize(7);
    // 圆柱参数：轴上一点(x,y,z)、轴方向(x,y,z)、半径
    cylinder_coeff.values[0] = 2.0f;  // 点 x
    cylinder_coeff.values[1] = 0.0f;  // 点 y
    cylinder_coeff.values[2] = 0.0f;  // 点 z
    cylinder_coeff.values[3] = 0.0f;  // 方向 x
    cylinder_coeff.values[4] = 0.0f;  // 方向 y
    cylinder_coeff.values[5] = 1.0f;  // 方向 z
    cylinder_coeff.values[6] = 0.3f;  // 半径
    //container.addCylinder("cylinder1", cylinder_coeff, 0, 0, 255);

    // 4. 添加平面 - 灰色，z=0平面
    pcl::ModelCoefficients plane_coeff;
    plane_coeff.values.resize(4);
    // 平面方程: ax + by + cz + d = 0
    plane_coeff.values[0] = 0.0f;  // a
    plane_coeff.values[1] = 0.0f;  // b
    plane_coeff.values[2] = 1.0f;  // c
    plane_coeff.values[3] = -1.0f; // d (z = 1)
    container.addPlane("plane1", plane_coeff, 128, 128, 128);

    // 5. 添加圆环 - 橙色
    container.addTorus("torus1", 1.5, 0.3, 255, 128, 0);

    // 6. 添加箭头 - 黄色，从(-1,0,1)指向(1,0,1)
    pcl::PointXYZ arrow_start(-1.0f, 0.0f, 1.0f);
    pcl::PointXYZ arrow_end(1.0f, 0.0f, 1.0f);
    container.addArrow("arrow1", arrow_start, arrow_end, 255, 255, 0);

    // 7. 添加3D文本 - 白色
    pcl::PointXYZ text_pos(0.0f, 0.0f, 2.0f);
    container.addText3D("text3d1", "PCL Shapes", text_pos, 0.2, 1.0, 1.0, 1.0);

    // 8. 添加2D屏幕文本 - 白色
    container.addText("text2d1", "几何形状演示", 10, 10, 16, 1.0, 1.0, 1.0);

    std::cout << "\n=== 几何形状演示 ===\n";
    std::cout << "已添加以下形状：\n";
    std::cout << "  - sphere1: 红色球体 (原点)\n";
    std::cout << "  - cube1: 绿色立方体\n";
    std::cout << "  - cylinder1: 蓝色圆柱体\n";
    std::cout << "  - plane1: 灰色平面 (z=1)\n";
    std::cout << "  - torus1: 橙色圆环\n";
    std::cout << "  - arrow1: 黄色箭头\n";
    std::cout << "  - text3d1: 3D文本\n";
    std::cout << "  - text2d1: 2D文本\n";
    std::cout << "\n按 'q' 关闭窗口\n";

    // 显示可视化窗口
    container.show();

    // ==================== 删除形状示例 ====================
    // 如果需要删除某个形状，可以使用：
    // container.removeShape("sphere1");

    // 删除所有形状：
    // container.removeAllShapes();

    return 0;
}

