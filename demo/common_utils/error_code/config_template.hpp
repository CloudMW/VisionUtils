//
// 错误码配置模板 - 根据你的项目需求定制
//
// 使用方法：
// 1. 复制此文件并重命名（如：my_project_error_code.hpp）
// 2. 修改命名空间为你的项目
// 3. 添加/删除/修改错误类别和错误码
// 4. 调整错误码范围以避免冲突
//

#ifndef MY_PROJECT_ERROR_CODE_H
#define MY_PROJECT_ERROR_CODE_H

#include <system_error>
#include <string>
#include <stdexcept>

// ============================================================================
// 配置选项 - Configuration Options
// ============================================================================

// 是否启用Result模板类
#define ENABLE_RESULT_CLASS 1

// 是否启用异常类
#define ENABLE_EXCEPTION_CLASS 1

// 是否启用辅助宏
#define ENABLE_HELPER_MACROS 1

// 自定义命名空间
#define PROJECT_NAMESPACE my_project
#define ERROR_NAMESPACE errors

// ============================================================================
// 命名空间定义
// ============================================================================

namespace PROJECT_NAMESPACE {
    namespace ERROR_NAMESPACE {

        // ========================================================================
        // 错误码范围分配建议
        // ========================================================================
        // 0x0000 - 成功/无错误
        // 0x1001xx - 模块1错误
        // 0x1002xx - 模块2错误
        // 0x1003xx - 模块3错误
        // 0x1004xx - 模块4错误
        // ...
        // 0x10FFxx - 通用错误

        // ========================================================================
        // 错误码定义 - 根据你的项目需求修改
        // ========================================================================

        // 示例：文件系统错误
        enum class FileSystemCode {
            Ok = 0,
            FILE_NOT_FOUND = 0x100101,
            DIRECTORY_NOT_FOUND = 0x100102,
            ACCESS_DENIED = 0x100103,
            DISK_FULL = 0x100104,
            FILE_ALREADY_EXISTS = 0x100105
        };

        // 示例：网络错误
        enum class NetworkCode {
            Ok = 0,
            CONNECTION_FAILED = 0x100201,
            TIMEOUT = 0x100202,
            HOST_NOT_FOUND = 0x100203,
            PROTOCOL_ERROR = 0x100204,
            INVALID_RESPONSE = 0x100205
        };

        // 示例：数据库错误
        enum class DatabaseCode {
            Ok = 0,
            CONNECTION_FAILED = 0x100301,
            QUERY_FAILED = 0x100302,
            DUPLICATE_KEY = 0x100303,
            CONSTRAINT_VIOLATION = 0x100304,
            TRANSACTION_FAILED = 0x100305
        };

        // 通用错误
        enum class CommonCode {
            Ok = 0,
            UNKNOWN_ERROR = 0x10FF01,
            NOT_IMPLEMENTED = 0x10FF02,
            OUT_OF_MEMORY = 0x10FF03,
            INVALID_ARGUMENT = 0x10FF04,
            OPERATION_CANCELLED = 0x10FF05
        };

        // ========================================================================
        // Error Category 实现
        // ========================================================================

        class FileSystemErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "my_project::filesystem";
            }

            std::string message(int ev) const override {
                switch (static_cast<FileSystemCode>(ev)) {
                    case FileSystemCode::Ok:
                        return "Success";
                    case FileSystemCode::FILE_NOT_FOUND:
                        return "File not found";
                    case FileSystemCode::DIRECTORY_NOT_FOUND:
                        return "Directory not found";
                    case FileSystemCode::ACCESS_DENIED:
                        return "Access denied";
                    case FileSystemCode::DISK_FULL:
                        return "Disk full";
                    case FileSystemCode::FILE_ALREADY_EXISTS:
                        return "File already exists";
                    default:
                        return "Unknown filesystem error";
                }
            }
        };

        class NetworkErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "my_project::network";
            }

            std::string message(int ev) const override {
                switch (static_cast<NetworkCode>(ev)) {
                    case NetworkCode::Ok:
                        return "Success";
                    case NetworkCode::CONNECTION_FAILED:
                        return "Connection failed";
                    case NetworkCode::TIMEOUT:
                        return "Operation timeout";
                    case NetworkCode::HOST_NOT_FOUND:
                        return "Host not found";
                    case NetworkCode::PROTOCOL_ERROR:
                        return "Protocol error";
                    case NetworkCode::INVALID_RESPONSE:
                        return "Invalid server response";
                    default:
                        return "Unknown network error";
                }
            }
        };

        class DatabaseErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "my_project::database";
            }

            std::string message(int ev) const override {
                switch (static_cast<DatabaseCode>(ev)) {
                    case DatabaseCode::Ok:
                        return "Success";
                    case DatabaseCode::CONNECTION_FAILED:
                        return "Database connection failed";
                    case DatabaseCode::QUERY_FAILED:
                        return "Query execution failed";
                    case DatabaseCode::DUPLICATE_KEY:
                        return "Duplicate key violation";
                    case DatabaseCode::CONSTRAINT_VIOLATION:
                        return "Constraint violation";
                    case DatabaseCode::TRANSACTION_FAILED:
                        return "Transaction failed";
                    default:
                        return "Unknown database error";
                }
            }
        };

        class CommonErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "my_project::common";
            }

            std::string message(int ev) const override {
                switch (static_cast<CommonCode>(ev)) {
                    case CommonCode::Ok:
                        return "Success";
                    case CommonCode::UNKNOWN_ERROR:
                        return "Unknown error";
                    case CommonCode::NOT_IMPLEMENTED:
                        return "Feature not implemented";
                    case CommonCode::OUT_OF_MEMORY:
                        return "Out of memory";
                    case CommonCode::INVALID_ARGUMENT:
                        return "Invalid argument";
                    case CommonCode::OPERATION_CANCELLED:
                        return "Operation cancelled";
                    default:
                        return "Unknown common error";
                }
            }
        };

        // ========================================================================
        // Category 单例访问
        // ========================================================================

        inline const FileSystemErrorCategory& filesystem_category() {
            static FileSystemErrorCategory instance;
            return instance;
        }

        inline const NetworkErrorCategory& network_category() {
            static NetworkErrorCategory instance;
            return instance;
        }

        inline const DatabaseErrorCategory& database_category() {
            static DatabaseErrorCategory instance;
            return instance;
        }

        inline const CommonErrorCategory& common_category() {
            static CommonErrorCategory instance;
            return instance;
        }

        // ========================================================================
        // make_error_code 函数
        // ========================================================================

        inline std::error_code make_error_code(FileSystemCode e) {
            return {static_cast<int>(e), filesystem_category()};
        }

        inline std::error_code make_error_code(NetworkCode e) {
            return {static_cast<int>(e), network_category()};
        }

        inline std::error_code make_error_code(DatabaseCode e) {
            return {static_cast<int>(e), database_category()};
        }

        inline std::error_code make_error_code(CommonCode e) {
            return {static_cast<int>(e), common_category()};
        }

        // ========================================================================
        // Result 类（可选）
        // ========================================================================

        #if ENABLE_RESULT_CLASS

        template<typename T>
        class Result {
        private:
            T value_;
            std::error_code error_;
            bool has_value_;

        public:
            Result(const T& val) : value_(val), has_value_(true) {}
            Result(T&& val) : value_(std::move(val)), has_value_(true) {}
            Result(std::error_code ec) : error_(ec), has_value_(false) {}

            template<typename ErrorEnum>
            Result(ErrorEnum e) : error_(make_error_code(e)), has_value_(false) {}

            bool ok() const { return has_value_ && !error_; }
            explicit operator bool() const { return ok(); }

            const T& value() const {
                if (!has_value_) {
                    throw std::runtime_error("Result has no value: " + error_.message());
                }
                return value_;
            }

            T& value() {
                if (!has_value_) {
                    throw std::runtime_error("Result has no value: " + error_.message());
                }
                return value_;
            }

            const std::error_code& error() const { return error_; }

            T value_or(const T& default_value) const {
                return has_value_ ? value_ : default_value;
            }
        };

        template<>
        class Result<void> {
        private:
            std::error_code error_;

        public:
            Result() : error_() {}
            Result(std::error_code ec) : error_(ec) {}

            template<typename ErrorEnum>
            Result(ErrorEnum e) : error_(make_error_code(e)) {}

            bool ok() const { return !error_; }
            explicit operator bool() const { return ok(); }

            const std::error_code& error() const { return error_; }
        };

        #endif // ENABLE_RESULT_CLASS

        // ========================================================================
        // 异常类（可选）
        // ========================================================================

        #if ENABLE_EXCEPTION_CLASS

        class ProjectException : public std::system_error {
        public:
            template<typename ErrorEnum>
            explicit ProjectException(ErrorEnum e, const std::string& what_arg = "")
                : std::system_error(make_error_code(e), what_arg) {}

            ProjectException(std::error_code ec, const std::string& what_arg = "")
                : std::system_error(ec, what_arg) {}
        };

        #endif // ENABLE_EXCEPTION_CLASS

        // ========================================================================
        // 辅助宏（可选）
        // ========================================================================

        #if ENABLE_HELPER_MACROS

        #define PROJECT_CHECK(condition, error_code) \
            if (!(condition)) { \
                throw PROJECT_NAMESPACE::ERROR_NAMESPACE::ProjectException(error_code, \
                    std::string(__FILE__) + ":" + std::to_string(__LINE__)); \
            }

        #define PROJECT_RETURN_IF_ERROR(expr) \
            do { \
                auto _result = (expr); \
                if (!_result.ok()) { \
                    return _result.error(); \
                } \
            } while(0)

        #endif // ENABLE_HELPER_MACROS

    } // namespace ERROR_NAMESPACE
} // namespace PROJECT_NAMESPACE

// ============================================================================
// std::error_code 特化
// ============================================================================

namespace std {
    template<>
    struct is_error_code_enum<PROJECT_NAMESPACE::ERROR_NAMESPACE::FileSystemCode> : true_type {};

    template<>
    struct is_error_code_enum<PROJECT_NAMESPACE::ERROR_NAMESPACE::NetworkCode> : true_type {};

    template<>
    struct is_error_code_enum<PROJECT_NAMESPACE::ERROR_NAMESPACE::DatabaseCode> : true_type {};

    template<>
    struct is_error_code_enum<PROJECT_NAMESPACE::ERROR_NAMESPACE::CommonCode> : true_type {};
}

// ============================================================================
// 使用示例
// ============================================================================

/*

// 基本使用
using namespace my_project::errors;

// 方法1: std::error_code
std::error_code ec = make_error_code(FileSystemCode::FILE_NOT_FOUND);
if (ec) {
    std::cerr << ec.message() << "\n";
}

// 方法2: Result模式
Result<std::string> readFile(const std::string& path) {
    if (!exists(path)) {
        return FileSystemCode::FILE_NOT_FOUND;
    }
    return fileContent;
}

auto result = readFile("config.txt");
if (result.ok()) {
    std::cout << result.value() << "\n";
} else {
    std::cerr << result.error().message() << "\n";
}

// 方法3: 异常
try {
    PROJECT_CHECK(data != nullptr, CommonCode::INVALID_ARGUMENT);
} catch (const ProjectException& e) {
    std::cerr << e.what() << "\n";
}

*/

#endif // MY_PROJECT_ERROR_CODE_H

