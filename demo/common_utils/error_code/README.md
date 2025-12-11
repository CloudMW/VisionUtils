# PCDL Error Code Library

一个轻量级、易于移植的C++错误处理库，支持多种错误码分类、Result模式和异常处理。

## 特性

✅ **多种错误类别** - IO、算法、可视化、通用错误分类管理  
✅ **标准兼容** - 完全兼容 `std::error_code` 和 `std::system_error`  
✅ **Result模式** - 类似Rust的Result类型，支持函数式错误处理  
✅ **零依赖** - 只需C++11标准库  
✅ **Header-only** - 单头文件，易于集成  
✅ **易于扩展** - 简单添加自定义错误类别  
✅ **性能优良** - 零成本抽象，无额外运行时开销  

## 快速开始

### 1. 包含头文件

```cpp
#include "pcdl_error_code.hpp"
using namespace pcdl::error_code;
```

### 2. 基本使用

```cpp
// 方法1: 使用 std::error_code
std::error_code ec = make_error_code(IOCode::FILE_NOT_FOUND);
if (ec) {
    std::cerr << "Error: " << ec.message() << "\n";
}

// 方法2: 使用 Result 模式
Result<std::vector<double>> loadData(const std::string& filename) {
    if (!fileExists(filename)) {
        return IOCode::FILE_NOT_FOUND;
    }
    // ... 读取数据
    return data;
}

auto result = loadData("data.txt");
if (result.ok()) {
    auto data = result.value();
} else {
    std::cerr << result.error().message() << "\n";
}

// 方法3: 使用异常
void processData(const std::vector<double>& data) {
    PCDL_CHECK(!data.empty(), AlgoCode::EMPTY_POINT_CLOUD);
    // ... 处理数据
}
```

## 文件说明

### 核心文件

- **`pcdl_error_code.hpp`** - 错误库头文件（唯一必需的文件）

### 示例和文档

- **`example_usage.cpp`** - 8个完整的使用示例
- **`test_error_code.cpp`** - 单元测试
- **`PORTING_GUIDE.md`** - 详细的移植和配置指南
- **`QUICK_REFERENCE.md`** - 快速参考手册
- **`CMakeLists.txt`** - CMake构建配置

## 编译和运行

### 使用CMake

```bash
# 配置项目
cmake -B build -S .

# 编译
cmake --build build

# 运行示例
./build/error_code_example

# 运行测试
./build/error_code_test
# 或
ctest --test-dir build
```

### 直接编译

```bash
# Linux/macOS (GCC/Clang)
g++ -std=c++11 -I../../pcdl/error_code example_usage.cpp -o example
./example

# Windows (MSVC)
cl /std:c++11 /I..\..\pcdl\error_code example_usage.cpp
example.exe
```

## 使用示例

### 示例1: 基本错误处理

```cpp
std::error_code ec = make_error_code(IOCode::FILE_NOT_FOUND);
if (ec) {
    std::cerr << "错误: " << ec.message() << "\n";
    std::cerr << "类别: " << ec.category().name() << "\n";
    std::cerr << "代码: 0x" << std::hex << ec.value() << "\n";
}
```

### 示例2: Result模式

```cpp
Result<int> divide(int a, int b) {
    if (b == 0) {
        return AlgoCode::INVALID_PARAMETER;
    }
    return a / b;
}

auto result = divide(10, 2);
if (result.ok()) {
    std::cout << "结果: " << result.value() << "\n";
} else {
    std::cerr << "错误: " << result.error().message() << "\n";
}
```

### 示例3: 异常处理

```cpp
try {
    if (points.empty()) {
        throw PCDLException(AlgoCode::EMPTY_POINT_CLOUD, "点云为空");
    }
    // 处理点云...
} catch (const PCDLException& e) {
    std::cerr << "异常: " << e.what() << "\n";
    std::cerr << "错误码: 0x" << std::hex << e.code().value() << "\n";
}
```

### 示例4: 错误传播

```cpp
Result<Output> pipeline() {
    auto step1 = firstOperation();
    if (!step1.ok()) return step1.error();
    
    auto step2 = secondOperation(step1.value());
    if (!step2.ok()) return step2.error();
    
    return finalOperation(step2.value());
}
```

## 错误类别

### IOCode (0x8001xx) - IO相关错误
- `FILE_NOT_FOUND` - 文件未找到
- `FILE_OPEN_FAILED` - 文件打开失败
- `FILE_READ_FAILED` - 文件读取失败
- `FILE_WRITE_FAILED` - 文件写入失败
- `INVALID_FORMAT` - 无效的文件格式
- `PERMISSION_DENIED` - 权限拒绝
- `PATH_NOT_FOUND` - 路径不存在

### AlgoCode (0x8002xx) - 算法相关错误
- `EMPTY_POINT_CLOUD` - 空点云
- `INVALID_PARAMETER` - 无效参数
- `SEGMENTATION_FAILED` - 分割失败
- `CLUSTERING_FAILED` - 聚类失败
- `FITTING_FAILED` - 拟合失败
- `CONVERGENCE_FAILED` - 收敛失败
- `INSUFFICIENT_POINTS` - 点数不足
- `DIMENSION_MISMATCH` - 维度不匹配

### VisualizationCode (0x8003xx) - 可视化相关错误
- `INIT_FAILED` - 初始化失败
- `RENDER_FAILED` - 渲染失败
- `WINDOW_CREATE_FAILED` - 窗口创建失败
- `INVALID_COLOR_MODE` - 无效的颜色模式

### CommonCode (0x8000xx) - 通用错误
- `UNKNOWN_ERROR` - 未知错误
- `NOT_IMPLEMENTED` - 未实现
- `OUT_OF_MEMORY` - 内存不足
- `INVALID_STATE` - 无效状态
- `TIMEOUT` - 超时
- `CANCELLED` - 已取消

## 扩展和定制

### 添加自定义错误类别

详见 `PORTING_GUIDE.md` 中的完整说明。简要步骤：

1. 定义错误枚举
2. 创建ErrorCategory类
3. 提供单例访问函数
4. 实现make_error_code函数
5. 注册到std::is_error_code_enum

示例：
```cpp
enum class MyCode {
    Ok = 0,
    MY_ERROR = 0x800501
};

class MyErrorCategory : public std::error_category {
    const char* name() const noexcept override { return "my_module"; }
    std::string message(int ev) const override { /* ... */ }
};

inline std::error_code make_error_code(MyCode e) {
    static MyErrorCategory cat;
    return {static_cast<int>(e), cat};
}

namespace std {
    template<> struct is_error_code_enum<MyCode> : true_type {};
}
```

## 最佳实践

### 何时使用Result vs 异常

**使用Result：**
- ✅ 预期的错误（如文件未找到）
- ✅ 性能关键代码
- ✅ 需要显式错误传播
- ✅ 不使用异常的项目

**使用异常：**
- ✅ 不可恢复的错误
- ✅ 构造函数中的错误
- ✅ 错误很少发生的情况
- ✅ 需要跨越多层调用栈

### 错误处理模式

```cpp
// 模式1: 立即检查
if (!result.ok()) {
    handleError(result.error());
    return;
}

// 模式2: 提供默认值
auto data = result.value_or(defaultData);

// 模式3: 使用宏简化
PCDL_RETURN_IF_ERROR(result);

// 模式4: 转换为异常
if (!result.ok()) {
    throw PCDLException(result.error());
}
```

## 性能

- **零成本抽象** - Result<T>在编译优化后无额外开销
- **轻量级** - std::error_code只有两个指针大小
- **无动态分配** - 所有类别对象都是单例
- **比异常快** - 对于预期错误，避免栈展开开销

## 兼容性

- **C++11** 或更高版本
- **编译器**: GCC 4.8+, Clang 3.3+, MSVC 2015+
- **平台**: Windows, Linux, macOS, Unix

## 许可证

根据项目需求选择合适的许可证。

## 文档

- 📘 [完整移植指南](PORTING_GUIDE.md)
- 📙 [快速参考](QUICK_REFERENCE.md)
- 💻 [使用示例](example_usage.cpp)
- 🧪 [单元测试](test_error_code.cpp)

## 更多示例

运行示例程序查看8个完整的使用场景：

```bash
./build/error_code_example
```

输出示例：
```
=== Example 1: Basic Error Code Usage ===
Error occurred: File not found
Error category: pcdl::io
Error value: 0x800101

=== Example 2: Result Pattern ===
Failed to read file: File not found
Using 3 points (with fallback)

...
```

## 测试

运行单元测试验证功能：

```bash
./build/error_code_test
```

预期输出：
```
========================================
PCDL Error Code Library - Unit Tests
========================================

Running: error_code_creation... PASSED
Running: error_code_message... PASSED
Running: error_category... PASSED
...

========================================
Test Summary:
  Passed: 18
  Failed: 0
  Total:  18
========================================
```

## 贡献

欢迎提交Issue和Pull Request！

## 常见问题

**Q: 如何在DLL边界使用？**  
A: 使用错误码值（int）而不是error_code对象，详见PORTING_GUIDE.md

**Q: 如何添加本地化支持？**  
A: 创建支持locale的Category类，详见PORTING_GUIDE.md

**Q: 性能如何？**  
A: Result<T>是零成本抽象，error_code很轻量，比异常快

**Q: 可以只使用一部分功能吗？**  
A: 可以，根据需要注释掉不需要的错误类别或Result类

## 联系方式

有问题或建议？请联系项目维护者。

