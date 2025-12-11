//
// 对比示例：使用 ErrorCategory vs 不使用 ErrorCategory
//
// 本文件展示两种错误处理方法的优缺点
//

#include <iostream>
#include <string>
#include <system_error>

// ============================================================================
// 方法1: 不使用 ErrorCategory - 简单但功能有限
// ============================================================================

namespace simple_approach {

    // 简单的错误码枚举
    enum class ErrorCode {
        Ok = 0,
        FILE_NOT_FOUND = 1,
        INVALID_FORMAT = 2,
        EMPTY_DATA = 3,
        UNKNOWN_ERROR = 99
    };

    // 简单的错误消息函数
    inline const char* getErrorMessage(ErrorCode code) {
        switch (code) {
            case ErrorCode::Ok:
                return "Success";
            case ErrorCode::FILE_NOT_FOUND:
                return "File not found";
            case ErrorCode::INVALID_FORMAT:
                return "Invalid format";
            case ErrorCode::EMPTY_DATA:
                return "Empty data";
            case ErrorCode::UNKNOWN_ERROR:
            default:
                return "Unknown error";
        }
    }

    // 使用示例
    void example() {
        std::cout << "\n=== Simple Approach (不使用 ErrorCategory) ===\n";

        ErrorCode ec = ErrorCode::FILE_NOT_FOUND;

        // ❌ 不能与 std::error_code 集成
        // std::error_code std_ec = ec;  // 编译错误！

        // ✅ 可以简单使用
        if (ec != ErrorCode::Ok) {
            std::cerr << "Error: " << getErrorMessage(ec) << "\n";
        }

        // ❌ 不能使用 std::system_error
        // throw std::system_error(ec);  // 编译错误！

        // ✅ 只能抛出普通异常
        if (ec != ErrorCode::Ok) {
            throw std::runtime_error(getErrorMessage(ec));
        }
    }

    // 优点：
    // + 实现简单，代码量少
    // + 学习成本低
    // + 适合小项目或临时使用
    // + 无需理解 std::error_category

    // 缺点：
    // - 不能与标准库 std::error_code 集成
    // - 不能使用 std::system_error
    // - 无法区分不同模块的错误（都是int）
    // - 跨DLL边界可能有问题
    // - 无法与其他使用 std::error_code 的库互操作
}

// ============================================================================
// 方法2: 使用 ErrorCategory - 功能完整，与标准库集成
// ============================================================================

namespace category_approach {

    // 错误码枚举（相同）
    enum class ErrorCode {
        Ok = 0,
        FILE_NOT_FOUND = 1,
        INVALID_FORMAT = 2,
        EMPTY_DATA = 3,
        UNKNOWN_ERROR = 99
    };

    // 自定义 ErrorCategory
    class MyErrorCategory : public std::error_category {
    public:
        const char* name() const noexcept override {
            return "my_module";
        }

        std::string message(int ev) const override {
            switch (static_cast<ErrorCode>(ev)) {
                case ErrorCode::Ok:
                    return "Success";
                case ErrorCode::FILE_NOT_FOUND:
                    return "File not found";
                case ErrorCode::INVALID_FORMAT:
                    return "Invalid format";
                case ErrorCode::EMPTY_DATA:
                    return "Empty data";
                case ErrorCode::UNKNOWN_ERROR:
                default:
                    return "Unknown error";
            }
        }
    };

    // 单例访问
    inline const MyErrorCategory& my_category() {
        static MyErrorCategory instance;
        return instance;
    }

    // make_error_code 函数
    inline std::error_code make_error_code(ErrorCode e) {
        return {static_cast<int>(e), my_category()};
    }

    // 使用示例
    void example() {
        std::cout << "\n=== Category Approach (使用 ErrorCategory) ===\n";

        ErrorCode ec = ErrorCode::FILE_NOT_FOUND;

        // ✅ 可以转换为 std::error_code
        std::error_code std_ec = make_error_code(ec);

        // ✅ 可以使用标准库功能
        if (std_ec) {
            std::cerr << "Error: " << std_ec.message() << "\n";
            std::cerr << "Category: " << std_ec.category().name() << "\n";
        }

        // ✅ 可以使用 std::system_error
        if (std_ec) {
            throw std::system_error(std_ec, "Additional context");
        }

        // ✅ 可以比较不同类别的错误
        std::error_code other_ec = std::make_error_code(std::errc::no_such_file_or_directory);
        if (std_ec.category() != other_ec.category()) {
            std::cout << "Different error categories\n";
        }
    }

    // 优点：
    // + 完全兼容 std::error_code
    // + 可以使用 std::system_error
    // + 支持错误类别区分
    // + 可以与标准库和第三方库互操作
    // + 支持更丰富的错误信息
    // + 类型安全

    // 缺点：
    // - 需要更多样板代码
    // - 学习成本稍高
    // - 对简单场景可能过于复杂
}

// ============================================================================
// 方法3: 混合方法 - 简单接口 + Category支持
// ============================================================================

namespace hybrid_approach {

    enum class ErrorCode {
        Ok = 0,
        FILE_NOT_FOUND = 1,
        INVALID_FORMAT = 2,
        EMPTY_DATA = 3
    };

    // 简单的消息函数（向后兼容）
    inline const char* getErrorMessage(ErrorCode code) {
        switch (code) {
            case ErrorCode::Ok: return "Success";
            case ErrorCode::FILE_NOT_FOUND: return "File not found";
            case ErrorCode::INVALID_FORMAT: return "Invalid format";
            case ErrorCode::EMPTY_DATA: return "Empty data";
            default: return "Unknown error";
        }
    }

    // ErrorCategory（可选的高级功能）
    class MyErrorCategory : public std::error_category {
    public:
        const char* name() const noexcept override { return "hybrid"; }

        std::string message(int ev) const override {
            return getErrorMessage(static_cast<ErrorCode>(ev));
        }
    };

    inline const MyErrorCategory& my_category() {
        static MyErrorCategory instance;
        return instance;
    }

    inline std::error_code make_error_code(ErrorCode e) {
        return {static_cast<int>(e), my_category()};
    }

    // 使用示例
    void example() {
        std::cout << "\n=== Hybrid Approach (混合方法) ===\n";

        ErrorCode ec = ErrorCode::FILE_NOT_FOUND;

        // ✅ 简单使用（不需要了解 error_code）
        if (ec != ErrorCode::Ok) {
            std::cerr << "Simple: " << getErrorMessage(ec) << "\n";
        }

        // ✅ 高级使用（需要时可以用 error_code）
        std::error_code std_ec = make_error_code(ec);
        if (std_ec) {
            std::cerr << "Advanced: " << std_ec.message() << "\n";
        }
    }

    // 优点：
    // + 向后兼容简单方法
    // + 提供高级功能给需要的用户
    // + 灵活性高

    // 缺点：
    // - 两套API可能造成混乱
    // - 需要维护两种接口
}

// ============================================================================
// 实际使用场景对比
// ============================================================================

namespace practical_comparison {

    // 场景1: 函数返回错误
    namespace simple {
        enum class ErrorCode { Ok, Error };
        const char* getMessage(ErrorCode e) { return e == ErrorCode::Ok ? "OK" : "Error"; }

        ErrorCode loadFile(const std::string& path) {
            return ErrorCode::Error;
        }

        void use() {
            auto ec = loadFile("test.txt");
            if (ec != ErrorCode::Ok) {
                std::cerr << getMessage(ec) << "\n";
            }
        }
    }

    namespace category {
        enum class ErrorCode { Ok, Error };
        class Cat : public std::error_category {
        public:
            const char* name() const noexcept override { return "cat"; }
            std::string message(int) const override { return "Error"; }
        };
        inline const Cat& cat() { static Cat c; return c; }
        inline std::error_code make_error_code(ErrorCode e) {
            return {static_cast<int>(e), cat()};
        }

        std::error_code loadFile(const std::string& path) {
            return make_error_code(ErrorCode::Error);
        }

        void use() {
            auto ec = loadFile("test.txt");
            if (ec) {
                std::cerr << ec.message() << "\n";
                // 可以传递给其他接受 std::error_code 的函数
                processError(ec);
            }
        }

        void processError(const std::error_code& ec) {
            // 可以统一处理所有错误
            std::cerr << ec.category().name() << ": " << ec.message() << "\n";
        }
    }

    // 场景2: 多模块错误
    void demonstrate_multi_module() {
        std::cout << "\n=== Multi-Module Error Handling ===\n";

        // 使用简单方法：无法区分来源
        std::cout << "Simple approach:\n";
        int error_from_module_a = 1;  // FILE_NOT_FOUND
        int error_from_module_b = 1;  // NETWORK_ERROR
        // 问题：都是1，无法区分是文件错误还是网络错误
        std::cout << "  Error A: " << error_from_module_a << "\n";
        std::cout << "  Error B: " << error_from_module_b << "\n";
        std::cout << "  Cannot distinguish! Both are 1\n";

        // 使用Category方法：可以清晰区分
        std::cout << "\nCategory approach:\n";
        // 每个模块有自己的category
        // ec_a.category().name() == "module_a"
        // ec_b.category().name() == "module_b"
        std::cout << "  Errors can be distinguished by category\n";
        std::cout << "  Error A category: module_a\n";
        std::cout << "  Error B category: module_b\n";
    }
}

// ============================================================================
// 性能对比
// ============================================================================

namespace performance_comparison {

    void demonstrate() {
        std::cout << "\n=== Performance Comparison ===\n";

        std::cout << "Simple approach:\n";
        std::cout << "  sizeof(int): " << sizeof(int) << " bytes\n";
        std::cout << "  Cost: 直接整数比较\n";

        std::cout << "\nCategory approach:\n";
        std::cout << "  sizeof(std::error_code): " << sizeof(std::error_code) << " bytes\n";
        std::cout << "  Cost: 两个指针（错误值 + category指针）\n";
        std::cout << "  Note: Category对象是单例，无额外内存\n";

        std::cout << "\n结论：\n";
        std::cout << "  - 内存开销：error_code 略大（多一个指针）\n";
        std::cout << "  - 性能开销：几乎可以忽略不计\n";
        std::cout << "  - 在现代硬件上，差异微不足道\n";
    }
}

// ============================================================================
// 主函数
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "ErrorCategory 使用对比\n";
    std::cout << "========================================\n";

    try {
        // 注意：这些示例会抛出异常，用于演示
        // simple_approach::example();
    } catch (const std::exception& e) {
        std::cerr << "Simple approach exception: " << e.what() << "\n";
    }

    try {
        // category_approach::example();
    } catch (const std::exception& e) {
        std::cerr << "Category approach exception: " << e.what() << "\n";
    }

    hybrid_approach::example();
    practical_comparison::demonstrate_multi_module();
    performance_comparison::demonstrate();

    std::cout << "\n========================================\n";
    std::cout << "总结和建议\n";
    std::cout << "========================================\n";
    std::cout << "\n推荐使用 ErrorCategory 如果：\n";
    std::cout << "  ✓ 需要与标准库集成\n";
    std::cout << "  ✓ 有多个错误来源/模块\n";
    std::cout << "  ✓ 需要与第三方库互操作\n";
    std::cout << "  ✓ 长期维护的项目\n";
    std::cout << "  ✓ 跨DLL边界使用\n";

    std::cout << "\n可以使用简单方法如果：\n";
    std::cout << "  ✓ 非常简单的项目\n";
    std::cout << "  ✓ 原型开发/临时代码\n";
    std::cout << "  ✓ 不需要与std库集成\n";
    std::cout << "  ✓ 错误来源单一\n";

    std::cout << "\n关键区别：\n";
    std::cout << "  1. 类型安全：Category方法有类别区分\n";
    std::cout << "  2. 标准兼容：Category可用于std::system_error\n";
    std::cout << "  3. 互操作性：Category可与其他库配合\n";
    std::cout << "  4. 可扩展性：Category更易于扩展\n";
    std::cout << "  5. 学习成本：简单方法更易理解\n";

    std::cout << "\n========================================\n";

    return 0;
}

