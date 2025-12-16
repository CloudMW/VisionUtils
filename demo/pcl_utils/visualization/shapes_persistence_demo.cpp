/**
 * @file shapes_persistence_demo.cpp
 * @brief 演示几何形状持久化功能
 *
 * 演示如何使用 PCLVisContainer 的几何形状持久化功能。
 * 关闭窗口后再次调用 show() 时，所有形状都会保留。
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
#include "pcl_utils/visualization/pcl_utils_vis_container.hpp"

int main() {
    using namespace pcl_utils::visualization;

    // 创建可视化容器
    PCLVisContainer container;

    std::cout << "\n=== 几何形状持久化演示 ===\n";
    std::cout << "第一次显示窗口...\n\n";

    // ==================== 添加几何形状 ====================

    // 添加球体
    pcl::PointXYZ sphere_center(0.0f, 0.0f, 0.0f);
    container.addSphere("sphere1", sphere_center, 0.5, 255, 0, 0);
    std::cout << "✓ 添加了红色球体\n";

    // 添加立方体
    container.addCube("cube1", -2.0, -1.0, -0.5, 0.5, -0.5, 0.5, 0, 255, 0);
    std::cout << "✓ 添加了绿色立方体\n";

    // 添加箭头
    pcl::PointXYZ arrow_start(-1.0f, 0.0f, 1.0f);
    pcl::PointXYZ arrow_end(1.0f, 0.0f, 1.0f);
    container.addArrow("arrow1", arrow_start, arrow_end, 255, 255, 0);
    std::cout << "✓ 添加了黄色箭头\n";

    // 添加圆环
    container.addTorus("torus1", 1.5, 0.3, 255, 128, 0);
    std::cout << "✓ 添加了橙色圆环\n";

    // 添加3D文本
    pcl::PointXYZ text_pos(0.0f, 0.0f, 2.0f);
    container.addText3D("text3d1", "Persistent Shapes", text_pos, 0.2, 1.0, 1.0, 1.0);
    std::cout << "✓ 添加了3D文本\n";

    // 添加2D文本
    container.addText("text2d1", "第一次显示 - 关闭窗口后将重新打开", 10, 10, 16, 1.0, 1.0, 1.0);
    std::cout << "✓ 添加了2D文本\n";

    std::cout << "\n请关闭可视化窗口以继续...\n";

    // 第一次显示
    container.show();

    std::cout << "\n=== 窗口已关闭 ===\n";
    std::cout << "现在修改 2D 文本并再次显示...\n";

    // 删除旧的2D文本，添加新的
    container.removeShape("text2d1");
    container.addText("text2d1", "第二次显示 - 所有形状都保留了！", 10, 10, 16, 0.0, 1.0, 0.0);
    std::cout << "✓ 更新了2D文本\n";

    // 添加一个新的球体
    pcl::PointXYZ sphere2_center(2.0f, 0.0f, 0.0f);
    container.addSphere("sphere2", sphere2_center, 0.3, 0, 0, 255);
    std::cout << "✓ 添加了新的蓝色球体\n";

    std::cout << "\n第二次显示窗口...\n";
    std::cout << "你会看到：\n";
    std::cout << "  - 所有之前的形状都还在（持久化成功）\n";
    std::cout << "  - 2D文本已更新\n";
    std::cout << "  - 新增了一个蓝色球体\n";
    std::cout << "\n请关闭窗口以退出...\n";

    // 第二次显示 - 所有形状都会保留
    container.show();

    std::cout << "\n=== 演示完成 ===\n";
    std::cout << "总结：\n";
    std::cout << "  ✓ 几何形状在 show() 调用之间保持持久化\n";
    std::cout << "  ✓ 可以在重新显示前添加/删除/修改形状\n";
    std::cout << "  ✓ 形状的所有参数（位置、大小、颜色等）都被保存\n";

    return 0;
}

