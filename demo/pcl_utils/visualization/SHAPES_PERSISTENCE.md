# 几何形状持久化功能说明

## 概述

`PCLVisContainer` 类现在支持几何形状的持久化存储，这意味着：
- 添加的几何形状会被保存在内存中
- 当窗口关闭后再次调用 `show()` 时，所有形状会自动恢复
- 形状的所有参数（位置、大小、颜色等）都会被完整保存

## 工作原理

### 1. 内部存储机制

每个几何形状在添加时，其参数会被保存到内部的 `shapes_` 容器中：

```cpp
struct ShapeData {
    std::string type;  // 形状类型：sphere, cube, cylinder, etc.
    std::any data;     // 形状的所有参数
};

std::map<std::string, ShapeData> shapes_;  // 持久化存储
```

### 2. 自动恢复

当调用 `show()` 后窗口被关闭，再次调用 `show()` 时：

1. 检测到窗口已停止 (`viewer_->wasStopped()`)
2. 调用 `reset()` 方法重建可视化器
3. 自动从 `shapes_` 容器恢复所有几何形状
4. 保持原有的位置、大小、颜色等所有属性

### 3. 形状生命周期

```
添加形状 → 保存到 shapes_ → 显示
                ↓
            窗口关闭
                ↓
         reset() 重建
                ↓
        从 shapes_ 恢复 → 再次显示
```

## 使用示例

### 基本用法

```cpp
PCLVisContainer container;

// 添加形状
container.addSphere("sphere1", center, 0.5, 255, 0, 0);
container.addCube("cube1", -1, 1, -1, 1, -1, 1, 0, 255, 0);

// 第一次显示
container.show();  // 用户关闭窗口

// 窗口关闭后，形状仍然保存在内存中

// 第二次显示 - 所有形状自动恢复
container.show();  // 之前的球体和立方体都还在！
```

### 在两次显示之间修改形状

```cpp
PCLVisContainer container;

// 添加初始形状
container.addSphere("sphere1", center, 0.5, 255, 0, 0);
container.addCube("cube1", -1, 1, -1, 1, -1, 1, 0, 255, 0);

// 第一次显示
container.show();

// 窗口关闭后，可以修改形状
container.removeShape("sphere1");  // 删除球体
container.addArrow("arrow1", p1, p2, 255, 255, 0);  // 添加箭头

// 第二次显示 - 只有立方体和箭头
container.show();
```

### 与点云结合使用

```cpp
PCLVisContainer container;

// 添加点云
auto cloud = /* ... */;
container.addPointCloud("cloud1", cloud, 2.0);

// 添加标注形状
container.addSphere("center_marker", center, 0.1, 255, 0, 0);
container.addText3D("label", "Center", center, 0.1);

// 显示
container.show();

// 窗口关闭后，点云和形状都会保留
container.show();  // 点云和形状都在
```

## 保存的形状参数

每种形状保存的参数如下：

| 形状类型 | 保存的参数 |
|---------|-----------|
| Sphere | 中心点, 半径, RGB颜色 |
| Cube | 6个边界值(x/y/z min/max), RGB颜色 |
| Cylinder | ModelCoefficients, RGB颜色 |
| Plane | ModelCoefficients, RGB颜色 |
| Torus | 环半径, 截面半径, RGB颜色 |
| Arrow | 起点, 终点, RGB颜色, 显示长度标志 |
| Text3D | 文本内容, 位置, 缩放, RGB颜色 |
| Text | 文本内容, 屏幕坐标, 字体大小, RGB颜色 |

## 删除形状

删除形状时会同时清除持久化数据：

```cpp
// 删除单个形状
container.removeShape("sphere1");  // 从显示和存储中都删除

// 删除所有形状
container.removeAllShapes();  // 清空所有形状和存储
```

## 注意事项

### 1. ID 必须唯一

```cpp
// ❌ 错误 - 重复ID
container.addSphere("shape1", center1, 0.5);
container.addCube("shape1", -1, 1, -1, 1, -1, 1);  // 警告：ID已存在

// ✓ 正确 - 使用不同ID
container.addSphere("sphere1", center1, 0.5);
container.addCube("cube1", -1, 1, -1, 1, -1, 1);
```

### 2. 内存管理

- 形状数据会一直保存在内存中，直到显式删除或容器销毁
- 如果不再需要某个形状，建议使用 `removeShape()` 删除
- 大量形状可能占用较多内存

### 3. 与点云一致

点云和几何形状使用相同的持久化机制，确保一致的用户体验：

```cpp
// 点云持久化
container.addPointCloud("cloud1", cloud);
container.show();  // 关闭
container.show();  // 点云还在 ✓

// 形状持久化
container.addSphere("sphere1", center, 0.5);
container.show();  // 关闭
container.show();  // 形状还在 ✓
```

## 实现细节

### ShapeData 结构

```cpp
struct ShapeData {
    std::string type;  // 形状类型标识
    std::any data;     // 使用 std::any 存储不同类型的参数元组
};
```

### 参数存储方式

不同形状使用不同的 tuple 类型存储参数：

```cpp
// 球体
std::tuple<pcl::PointXYZ, double, int, int, int>
//        中心点         半径    R   G   B

// 立方体
std::tuple<double, double, double, double, double, double, int, int, int>
//        x_min  x_max  y_min  y_max  z_min  z_max  R   G   B

// 箭头
std::tuple<pcl::PointXYZ, pcl::PointXYZ, int, int, int, bool>
//        起点           终点          R   G   B   显示长度
```

### reset() 中的恢复逻辑

```cpp
void reset() {
    // ... 重建 viewer ...
    
    // 恢复所有几何形状
    for (const auto &[shape_id, shape_data] : shapes_) {
        if (shape_data.type == "sphere") {
            auto params = std::any_cast<...>(shape_data.data);
            viewer_->addSphere(/* 使用保存的参数 */);
        }
        // ... 其他形状类型 ...
    }
}
```

## 性能考虑

- **添加形状**: O(1) - 直接添加到 map
- **删除形状**: O(log n) - map 查找
- **恢复形状**: O(n) - 遍历所有形状
- **内存**: 每个形状约 100-200 字节（取决于参数）

## 总结

✅ **优点**：
- 自动持久化，无需手动管理
- 与点云行为一致
- 参数完整保存
- 支持动态修改

✅ **适用场景**：
- 需要多次显示/关闭窗口的交互应用
- 需要在可视化之间保持标注的应用
- 点云分析中的几何标注

✅ **最佳实践**：
- 使用有意义的 ID 命名
- 及时删除不需要的形状
- 在显示之间灵活添加/删除形状

