# PCLVisContainer 几何形状功能使用说明

## 概述

`PCLVisContainer` 类现在支持添加和删除多种几何形状，包括：
- 球体 (Sphere)
- 立方体 (Cube)
- 圆柱体 (Cylinder)
- 平面 (Plane)
- 圆环 (Torus)
- 箭头 (Arrow)
- 3D文本 (Text3D)
- 2D屏幕文本 (Text)

## 基本用法

### 1. 球体 (Sphere)

```cpp
pcl::PointXYZ center(0.0f, 0.0f, 0.0f);
container.addSphere("sphere_id", center, radius, r, g, b);
```

参数说明：
- `id`: 球体标识符（唯一）
- `center`: 球心坐标 (pcl::PointXYZ)
- `radius`: 半径
- `r, g, b`: 颜色 (0-255)，默认红色 (255, 0, 0)

### 2. 立方体 (Cube)

```cpp
container.addCube("cube_id", 
                  x_min, x_max,
                  y_min, y_max, 
                  z_min, z_max,
                  r, g, b);
```

参数说明：
- `id`: 立方体标识符
- `x_min, x_max`: X轴范围
- `y_min, y_max`: Y轴范围
- `z_min, z_max`: Z轴范围
- `r, g, b`: 颜色 (0-255)，默认绿色 (0, 255, 0)

### 3. 圆柱体 (Cylinder)

```cpp
pcl::ModelCoefficients coeffs;
coeffs.values.resize(7);
// 轴上一点 (x, y, z)
coeffs.values[0] = x;
coeffs.values[1] = y;
coeffs.values[2] = z;
// 轴方向 (dx, dy, dz)
coeffs.values[3] = dx;
coeffs.values[4] = dy;
coeffs.values[5] = dz;
// 半径
coeffs.values[6] = radius;

container.addCylinder("cylinder_id", coeffs, r, g, b);
```

参数说明：
- `coefficients`: 圆柱体参数（轴上一点、轴方向、半径）
- `r, g, b`: 颜色 (0-255)，默认蓝色 (0, 0, 255)

### 4. 平面 (Plane)

```cpp
pcl::ModelCoefficients coeffs;
coeffs.values.resize(4);
// 平面方程: ax + by + cz + d = 0
coeffs.values[0] = a;
coeffs.values[1] = b;
coeffs.values[2] = c;
coeffs.values[3] = d;

container.addPlane("plane_id", coeffs, r, g, b);
```

参数说明：
- `coefficients`: 平面方程系数 (a, b, c, d)
- `r, g, b`: 颜色 (0-255)，默认灰色 (128, 128, 128)

### 5. 圆环 (Torus)

```cpp
container.addTorus("torus_id", ring_radius, cross_section_radius, r, g, b);
```

参数说明：
- `ring_radius`: 环的主半径，默认 1.0
- `cross_section_radius`: 截面半径，默认 0.2
- `r, g, b`: 颜色 (0-255)，默认橙色 (255, 128, 0)

### 6. 箭头 (Arrow)

```cpp
pcl::PointXYZ start(x1, y1, z1);
pcl::PointXYZ end(x2, y2, z2);
container.addArrow("arrow_id", start, end, r, g, b, display_length);
```

参数说明：
- `pt_start`: 起点
- `pt_end`: 终点
- `r, g, b`: 颜色 (0-255)，默认黄色 (255, 255, 0)
- `display_length`: 是否显示长度，默认 false

### 7. 3D文本 (Text3D)

```cpp
pcl::PointXYZ position(x, y, z);
container.addText3D("text3d_id", "文本内容", position, text_scale, r, g, b);
```

参数说明：
- `text`: 文本内容
- `position`: 3D空间位置
- `text_scale`: 文本缩放，默认 0.1
- `r, g, b`: 颜色 (0-1)，默认白色 (1.0, 1.0, 1.0)

### 8. 2D屏幕文本 (Text)

```cpp
container.addText("text2d_id", "文本内容", x, y, font_size, r, g, b);
```

参数说明：
- `text`: 文本内容
- `x, y`: 屏幕坐标（像素）
- `font_size`: 字体大小，默认 14
- `r, g, b`: 颜色 (0-1)，默认白色 (1.0, 1.0, 1.0)

## 删除形状

### 删除单个形状

```cpp
container.removeShape("shape_id");
```

### 删除所有形状

```cpp
container.removeAllShapes();
```

## 完整示例

```cpp
#include "pcl_utils/visualization/pcl_utils_vis_container.hpp"

int main() {
    pcl_utils::visualization::PCLVisContainer container;
    
    // 添加球体
    pcl::PointXYZ center(0, 0, 0);
    container.addSphere("sphere1", center, 0.5, 255, 0, 0);
    
    // 添加立方体
    container.addCube("cube1", -1, 0, -1, 0, -1, 0, 0, 255, 0);
    
    // 添加箭头
    pcl::PointXYZ start(0, 0, 0), end(1, 1, 1);
    container.addArrow("arrow1", start, end, 255, 255, 0);
    
    // 添加文本
    container.addText("info", "Geometry Demo", 10, 10);
    
    // 显示
    container.show();
    
    return 0;
}
```

## 注意事项

1. **ID 唯一性**: 每个形状的 ID 必须唯一，重复添加相同 ID 会显示警告并忽略
2. **颜色范围**: 
   - 几何形状使用 0-255 范围
   - 文本使用 0-1 范围
3. **参数化形状**: 圆柱体和平面需要使用 `pcl::ModelCoefficients` 定义参数
4. **内存管理**: 使用 VTK 智能指针自动管理内存，无需手动释放

## 形状跟踪

所有添加的形状 ID 都会被自动跟踪，防止重复添加。可以通过 `removeShape()` 或 `removeAllShapes()` 进行管理。

