# PCDL Error Code Library - 移植和配置指南

## 概述

PCDL错误码库是一个基于C++11标准的轻量级错误处理库，支持：
- ✅ 多种错误类别分类管理
- ✅ 与std::error_code完全集成
- ✅ Result模式（类似Rust）
- ✅ 异常和非异常两种风格
- ✅ 易于扩展和自定义

## 快速开始

### 1. 文件结构

```
your_project/
├── include/
│   └── pcdl_error_code.hpp    # 只需这一个头文件
└── examples/
    └── example_usage.cpp       # 使用示例
```

### 2. 最小依赖

- C++11 或更高版本
- 标准库：`<system_error>`, `<string>`, `<stdexcept>`

### 3. 基本使用

```cpp
#include "pcdl_error_code.hpp"
using namespace pcdl::error_code;

// 方法1：使用std::error_code
std::error_code ec = make_error_code(IOCode::FILE_NOT_FOUND);
if (ec) {
    std::cerr << "Error: " << ec.message() << "\n";
}

// 方法2：使用Result模式
Result<int> divide(int a, int b) {
    if (b == 0) {
        return AlgoCode::INVALID_PARAMETER;
    }
    return a / b;
}

auto result = divide(10, 0);
if (result.ok()) {
    std::cout << "Result: " << result.value() << "\n";
} else {
    std::cerr << "Error: " << result.error().message() << "\n";
}

// 方法3：使用异常
try {
    if (data.empty()) {
        throw PCDLException(AlgoCode::EMPTY_POINT_CLOUD, "No data");
    }
} catch (const PCDLException& e) {
    std::cerr << "Error: " << e.what() << "\n";
}
```

## 移植到其他项目

### 步骤1：复制头文件

只需复制 `pcdl_error_code.hpp` 到你的项目中。

### 步骤2：修改命名空间（可选）

如果需要更改命名空间以适应你的项目：

```cpp
// 原始
namespace pcdl {
    namespace error_code {
        // ...
    }
}

// 修改为你的项目命名空间
namespace your_project {
    namespace errors {
        // ...
    }
}
```

### 步骤3：自定义错误码

#### 添加新的错误类别

```cpp
// 1. 定义新的错误枚举
enum class NetworkCode {
    Ok = 0,
    CONNECTION_FAILED = 0x800401,
    TIMEOUT = 0x800402,
    INVALID_RESPONSE = 0x800403
};

// 2. 创建错误类别类
class NetworkErrorCategory : public std::error_category {
public:
    const char* name() const noexcept override {
        return "your_project::network";
    }

    std::string message(int ev) const override {
        switch (static_cast<NetworkCode>(ev)) {
            case NetworkCode::Ok:
                return "Success";
            case NetworkCode::CONNECTION_FAILED:
                return "Network connection failed";
            case NetworkCode::TIMEOUT:
                return "Network timeout";
            case NetworkCode::INVALID_RESPONSE:
                return "Invalid server response";
            default:
                return "Unknown network error";
        }
    }
};

// 3. 提供单例访问
inline const NetworkErrorCategory& network_category() {
    static NetworkErrorCategory instance;
    return instance;
}

// 4. 提供make_error_code函数
inline std::error_code make_error_code(NetworkCode e) {
    return {static_cast<int>(e), network_category()};
}

// 5. 注册到std（在std命名空间中）
namespace std {
    template<>
    struct is_error_code_enum<your_project::errors::NetworkCode> : true_type {};
}
```

#### 修改现有错误码

```cpp
// 在对应的枚举中添加新错误
enum class IOCode {
    Ok = 0,
    FILE_NOT_FOUND = 0x800101,
    // ... 现有错误
    
    // 添加新错误
    FILE_TOO_LARGE = 0x800108,
    CORRUPTED_DATA = 0x800109
};

// 在对应的Category类的message()中添加处理
std::string message(int ev) const override {
    switch (static_cast<IOCode>(ev)) {
        // ... 现有case
        
        case IOCode::FILE_TOO_LARGE:
            return "File size exceeds limit";
        case IOCode::CORRUPTED_DATA:
            return "Data is corrupted";
        default:
            return "Unknown IO error";
    }
}
```

### 步骤4：配置错误码范围

建议使用不同的前缀避免冲突：

```cpp
// 0x80 01 xx - IO错误
// 0x80 02 xx - 算法错误  
// 0x80 03 xx - 可视化错误
// 0x80 04 xx - 网络错误
// 0x80 05 xx - 数据库错误
// ...
```

## 配置选项

### 1. 禁用异常支持

如果你的项目不使用异常，可以移除或注释掉：

```cpp
// 注释掉这些部分
/*
class PCDLException : public std::system_error {
    // ...
};

#define PCDL_CHECK(condition, error_code) ...
*/
```

### 2. 简化Result类

如果不需要完整的Result类，可以使用简化版本：

```cpp
template<typename T>
struct SimpleResult {
    T value;
    std::error_code error;
    
    bool ok() const { return !error; }
};
```

### 3. 添加日志支持

```cpp
class LoggingErrorCategory : public IOErrorCategory {
public:
    std::string message(int ev) const override {
        auto msg = IOErrorCategory::message(ev);
        // 添加日志
        logToFile(msg);
        return msg;
    }
};
```

### 4. 添加本地化支持

```cpp
class LocalizedIOErrorCategory : public std::error_category {
private:
    std::string locale_;
    
public:
    LocalizedIOErrorCategory(const std::string& locale) : locale_(locale) {}
    
    std::string message(int ev) const override {
        if (locale_ == "zh_CN") {
            switch (static_cast<IOCode>(ev)) {
                case IOCode::FILE_NOT_FOUND:
                    return "文件未找到";
                case IOCode::FILE_OPEN_FAILED:
                    return "打开文件失败";
                // ...
            }
        }
        // 默认英文
        return IOErrorCategory().message(ev);
    }
};
```

## 最佳实践

### 1. 返回值 vs 异常

**使用Result返回值：**
- ✅ 预期的错误情况
- ✅ 性能敏感的代码
- ✅ 需要链式错误传播
- ✅ 不使用异常的项目

```cpp
Result<Data> loadData(const std::string& path) {
    if (!fileExists(path)) {
        return IOCode::FILE_NOT_FOUND;  // 预期的错误
    }
    return parseData(path);
}
```

**使用异常：**
- ✅ 不可恢复的错误
- ✅ 构造函数中的错误
- ✅ 跨越多层调用栈的错误
- ✅ 错误很少发生的情况

```cpp
Data loadDataOrThrow(const std::string& path) {
    if (!fileExists(path)) {
        throw PCDLException(IOCode::FILE_NOT_FOUND, path);
    }
    return parseData(path);
}
```

### 2. 错误处理模式

**模式1：立即检查**
```cpp
auto result = operation();
if (!result.ok()) {
    handleError(result.error());
    return;
}
processResult(result.value());
```

**模式2：链式传播**
```cpp
Result<OutputType> pipeline() {
    auto step1 = firstOperation();
    if (!step1.ok()) return step1.error();
    
    auto step2 = secondOperation(step1.value());
    if (!step2.ok()) return step2.error();
    
    return finalOperation(step2.value());
}
```

**模式3：使用宏简化**
```cpp
Result<OutputType> pipeline() {
    auto step1 = firstOperation();
    PCDL_RETURN_IF_ERROR(step1);
    
    auto step2 = secondOperation(step1.value());
    PCDL_RETURN_IF_ERROR(step2);
    
    return finalOperation(step2.value());
}
```

### 3. 错误信息增强

```cpp
Result<Data> loadWithContext(const std::string& path) {
    auto result = loadData(path);
    if (!result.ok()) {
        // 添加上下文信息
        throw PCDLException(result.error(), 
            "Failed to load: " + path + " at line " + std::to_string(__LINE__));
    }
    return result.value();
}
```

## 与其他错误处理库对比

| 特性 | PCDL Error Code | Boost.System | std::expected (C++23) |
|------|----------------|--------------|----------------------|
| C++版本 | C++11+ | C++03+ | C++23+ |
| 依赖 | 无 | Boost | 无 |
| Result模式 | ✅ | ❌ | ✅ |
| 异常支持 | ✅ | ✅ | ✅ |
| 自定义错误 | ✅ | ✅ | ✅ |
| 学习曲线 | 低 | 中 | 低 |

## 完整示例项目

参考 `demo/error_code/example_usage.cpp` 查看8个完整的使用示例：

1. 基本错误码使用
2. Result模式
3. 异常处理
4. 宏辅助
5. 错误传播
6. 错误类别比较
7. 自定义错误处理策略
8. 管道式处理

## 常见问题

### Q1: 如何在DLL边界使用？

使用std::error_code可以安全跨DLL边界传递，但要注意：
- Category对象必须在同一个DLL中
- 或者使用错误码值（int）而不是error_code对象

```cpp
// DLL导出函数
extern "C" __declspec(dllexport) 
int doSomething(/*args*/) {
    auto result = internalOperation();
    if (!result.ok()) {
        return result.error().value();  // 返回int值
    }
    return 0;  // 成功
}

// DLL使用者
int errorValue = doSomething(/*args*/);
if (errorValue != 0) {
    std::error_code ec = make_error_code(static_cast<IOCode>(errorValue));
    std::cerr << ec.message() << "\n";
}
```

### Q2: 如何支持多语言？

创建支持locale的Category类（见配置选项4）。

### Q3: 性能如何？

- std::error_code是轻量级的（通常是两个指针大小）
- Result<T>是零成本抽象（编译器优化后）
- 比异常快（无栈展开开销）

### Q4: 如何整合到现有代码？

逐步迁移：
1. 先在新代码中使用
2. 将旧的错误码映射到新的error_code
3. 使用适配器包装旧函数

```cpp
// 适配器示例
Result<Data> adaptOldFunction(const std::string& path) {
    int oldErrorCode = legacyLoadData(path, &data);
    if (oldErrorCode != 0) {
        return mapOldToNewError(oldErrorCode);
    }
    return data;
}
```

## 许可证

根据你的项目需求选择合适的许可证。建议使用MIT或Apache 2.0以便于移植。

## 支持和反馈

有问题或建议？请联系项目维护者或提交Issue。

