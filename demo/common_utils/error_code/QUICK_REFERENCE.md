# PCDL Error Code 快速参考

## 错误码类别

### IOCode (0x8001xx) - IO相关错误
```cpp
IOCode::Ok                   // 成功
IOCode::FILE_NOT_FOUND       // 文件未找到
IOCode::FILE_OPEN_FAILED     // 文件打开失败
IOCode::FILE_READ_FAILED     // 文件读取失败
IOCode::FILE_WRITE_FAILED    // 文件写入失败
IOCode::INVALID_FORMAT       // 无效的文件格式
IOCode::PERMISSION_DENIED    // 权限拒绝
IOCode::PATH_NOT_FOUND       // 路径不存在
```

### AlgoCode (0x8002xx) - 算法相关错误
```cpp
AlgoCode::Ok                 // 成功
AlgoCode::EMPTY_POINT_CLOUD  // 空点云
AlgoCode::INVALID_PARAMETER  // 无效参数
AlgoCode::SEGMENTATION_FAILED // 分割失败
AlgoCode::CLUSTERING_FAILED  // 聚类失败
AlgoCode::FITTING_FAILED     // 拟合失败
AlgoCode::CONVERGENCE_FAILED // 收敛失败
AlgoCode::INSUFFICIENT_POINTS // 点数不足
AlgoCode::DIMENSION_MISMATCH // 维度不匹配
```

### VisualizationCode (0x8003xx) - 可视化相关错误
```cpp
VisualizationCode::Ok                // 成功
VisualizationCode::INIT_FAILED       // 初始化失败
VisualizationCode::RENDER_FAILED     // 渲染失败
VisualizationCode::WINDOW_CREATE_FAILED // 窗口创建失败
VisualizationCode::INVALID_COLOR_MODE   // 无效的颜色模式
```

### CommonCode (0x8000xx) - 通用错误
```cpp
CommonCode::Ok               // 成功
CommonCode::UNKNOWN_ERROR    // 未知错误
CommonCode::NOT_IMPLEMENTED  // 未实现
CommonCode::OUT_OF_MEMORY    // 内存不足
CommonCode::INVALID_STATE    // 无效状态
CommonCode::TIMEOUT          // 超时
CommonCode::CANCELLED        // 已取消
```

## 核心API

### 创建错误码
```cpp
std::error_code ec = make_error_code(IOCode::FILE_NOT_FOUND);
std::error_code ec2 = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);
```

### 检查错误
```cpp
if (ec) {                           // 有错误
    std::cerr << ec.message();      // 获取错误消息
    std::cerr << ec.value();        // 获取错误值
    ec.category().name();           // 获取类别名称
}
```

### Result模式
```cpp
// 返回成功值
Result<int> func() {
    return 42;
}

// 返回错误
Result<int> func() {
    return IOCode::FILE_NOT_FOUND;
}

// 检查结果
auto result = func();
if (result.ok()) {
    int val = result.value();
} else {
    std::error_code ec = result.error();
}

// 提供默认值
int val = result.value_or(0);
```

### void类型的Result
```cpp
Result<void> func() {
    if (error) {
        return AlgoCode::INVALID_PARAMETER;
    }
    return Result<void>();  // 成功
}

auto result = func();
if (!result.ok()) {
    std::cerr << result.error().message();
}
```

### 异常处理
```cpp
// 抛出异常
throw PCDLException(IOCode::FILE_NOT_FOUND, "config.txt");

// 捕获异常
try {
    // ...
} catch (const PCDLException& e) {
    std::cerr << e.what();           // 完整消息
    std::error_code ec = e.code();   // 获取错误码
}
```

### 辅助宏
```cpp
// 条件检查
PCDL_CHECK(ptr != nullptr, CommonCode::INVALID_STATE);

// 错误传播
Result<int> func() {
    auto result = otherFunc();
    PCDL_RETURN_IF_ERROR(result);  // 如果有错误就返回
    // 继续处理...
}
```

## 使用模式

### 模式1：基本错误检查
```cpp
std::error_code ec = doSomething();
if (ec) {
    std::cerr << "Error: " << ec.message() << "\n";
    return;
}
```

### 模式2：返回Result
```cpp
Result<Data> loadData(const std::string& path) {
    if (!exists(path)) {
        return IOCode::FILE_NOT_FOUND;
    }
    Data data = parse(path);
    return data;
}
```

### 模式3：错误传播链
```cpp
Result<Output> pipeline() {
    auto r1 = step1();
    if (!r1.ok()) return r1.error();
    
    auto r2 = step2(r1.value());
    if (!r2.ok()) return r2.error();
    
    return step3(r2.value());
}
```

### 模式4：异常风格
```cpp
void processData() {
    PCDL_CHECK(!data.empty(), AlgoCode::EMPTY_POINT_CLOUD);
    PCDL_CHECK(data.size() >= 3, AlgoCode::INSUFFICIENT_POINTS);
    // 处理数据...
}
```

### 模式5：混合风格
```cpp
Result<Data> loadDataSafe(const std::string& path) {
    try {
        Data data = loadDataWithExceptions(path);
        return data;
    } catch (const PCDLException& e) {
        return e.code();
    } catch (const std::exception& e) {
        return CommonCode::UNKNOWN_ERROR;
    }
}
```

## 扩展示例

### 添加新错误类别
```cpp
// 1. 定义枚举
enum class MyCode {
    Ok = 0,
    MY_ERROR = 0x800501
};

// 2. 定义Category
class MyErrorCategory : public std::error_category {
public:
    const char* name() const noexcept override {
        return "my_module";
    }
    std::string message(int ev) const override {
        if (ev == 0x800501) return "My error occurred";
        return "Unknown";
    }
};

// 3. 单例函数
inline const MyErrorCategory& my_category() {
    static MyErrorCategory instance;
    return instance;
}

// 4. make_error_code
inline std::error_code make_error_code(MyCode e) {
    return {static_cast<int>(e), my_category()};
}

// 5. 注册（在namespace std中）
namespace std {
    template<>
    struct is_error_code_enum<MyCode> : true_type {};
}
```

### 自定义错误处理器
```cpp
class ErrorLogger {
public:
    template<typename T>
    static Result<T> logged(Result<T> result) {
        if (!result.ok()) {
            log(result.error());
        }
        return result;
    }
    
private:
    static void log(const std::error_code& ec) {
        // 记录到文件、数据库等
    }
};

// 使用
auto result = ErrorLogger::logged(loadData("file.txt"));
```

## 编译

### CMake
```cmake
cmake_minimum_required(VERSION 3.10)
project(my_project)
set(CMAKE_CXX_STANDARD 11)
include_directories(path/to/pcdl/error_code)
add_executable(my_app main.cpp)
```

### 直接编译
```bash
# GCC/Clang
g++ -std=c++11 -I/path/to/pcdl/error_code main.cpp -o my_app

# MSVC
cl /std:c++11 /I path\to\pcdl\error_code main.cpp
```

## 性能提示

1. **Result<T>是零成本抽象** - 编译器会优化掉额外开销
2. **error_code很轻量** - 只有两个指针大小
3. **避免不必要的异常** - 对于预期错误用Result
4. **使用移动语义** - Result支持移动构造
5. **Category是单例** - 无额外内存开销

## 调试技巧

### 打印错误详情
```cpp
void printError(const std::error_code& ec) {
    std::cerr << "Error Details:\n"
              << "  Category: " << ec.category().name() << "\n"
              << "  Value: 0x" << std::hex << ec.value() << std::dec << "\n"
              << "  Message: " << ec.message() << "\n";
}
```

### 错误栈追踪
```cpp
class ErrorStack {
    std::vector<std::error_code> stack_;
public:
    void push(std::error_code ec) { stack_.push_back(ec); }
    void print() {
        for (const auto& ec : stack_) {
            std::cerr << ec.category().name() << ": " 
                      << ec.message() << "\n";
        }
    }
};
```

## 常见陷阱

❌ **不要这样做：**
```cpp
Result<int> func() {
    int value;
    return value;  // value未初始化！
}
```

✅ **应该这样：**
```cpp
Result<int> func() {
    int value = 42;
    return value;
}
```

❌ **不要忽略返回值：**
```cpp
auto result = loadData();
// 忘记检查result.ok()
auto data = result.value();  // 可能抛异常！
```

✅ **应该检查：**
```cpp
auto result = loadData();
if (!result.ok()) {
    handleError(result.error());
    return;
}
auto data = result.value();  // 安全
```

## 更多资源

- 完整示例：`demo/error_code/example_usage.cpp`
- 移植指南：`demo/error_code/PORTING_GUIDE.md`
- 头文件：`pcdl/error_code/pcdl_error_code.hpp`

