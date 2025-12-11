//
// Created by Mengfanyong on 2025/12/9.
//
// PCDL Error Code Library - Portable and Configurable Error Handling
//

#ifndef POINTCLOUDDOCKLIB_PCDL_ERROR_CODE_H
#define POINTCLOUDDOCKLIB_PCDL_ERROR_CODE_H

#include <system_error>
#include <string>
#include <stdexcept>

namespace pcdl {
    namespace error_code {

        // ============================================================================
        // 错误代码枚举 - Error Code Enumerations
        // ============================================================================

        // IO 相关错误 (0x8001xx)
        enum class IOCode {
            Ok = 0,
            FILE_NOT_FOUND = 0x800101,       // 文件未找到
            FILE_OPEN_FAILED = 0x800102,     // 文件打开失败
            FILE_READ_FAILED = 0x800103,     // 文件读取失败
            FILE_WRITE_FAILED = 0x800104,    // 文件写入失败
            INVALID_FORMAT = 0x800105,       // 无效的文件格式
            PERMISSION_DENIED = 0x800106,    // 权限拒绝
            PATH_NOT_FOUND = 0x800107        // 路径不存在
        };

        // 算法相关错误 (0x8002xx)
        enum class AlgoCode {
            Ok = 0,
            EMPTY_POINT_CLOUD = 0x800201,    // 空点云
            INVALID_PARAMETER = 0x800202,     // 无效参数
            SEGMENTATION_FAILED = 0x800203,   // 分割失败
            CLUSTERING_FAILED = 0x800204,     // 聚类失败
            FITTING_FAILED = 0x800205,        // 拟合失败
            CONVERGENCE_FAILED = 0x800206,    // 收敛失败
            INSUFFICIENT_POINTS = 0x800207,   // 点数不足
            DIMENSION_MISMATCH = 0x800208     // 维度不匹配
        };

        // 可视化相关错误 (0x8003xx)
        enum class VisualizationCode {
            Ok = 0,
            INIT_FAILED = 0x800301,           // 初始化失败
            RENDER_FAILED = 0x800302,         // 渲染失败
            WINDOW_CREATE_FAILED = 0x800303,  // 窗口创建失败
            INVALID_COLOR_MODE = 0x800304     // 无效的颜色模式
        };

        // 通用错误 (0x8000xx)
        enum class CommonCode {
            Ok = 0,
            UNKNOWN_ERROR = 0x800001,         // 未知错误
            NOT_IMPLEMENTED = 0x800002,       // 未实现
            OUT_OF_MEMORY = 0x800003,         // 内存不足
            INVALID_STATE = 0x800004,         // 无效状态
            TIMEOUT = 0x800005,               // 超时
            CANCELLED = 0x800006              // 已取消
        };

        // ============================================================================
        // Error Category 实现 - Error Category Implementations
        // ============================================================================

        // IO 错误类别
        class IOErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "pcdl::io";
            }

            std::string message(int ev) const override {
                switch (static_cast<IOCode>(ev)) {
                    case IOCode::Ok:
                        return "Success";
                    case IOCode::FILE_NOT_FOUND:
                        return "File not found";
                    case IOCode::FILE_OPEN_FAILED:
                        return "Failed to open file";
                    case IOCode::FILE_READ_FAILED:
                        return "Failed to read file";
                    case IOCode::FILE_WRITE_FAILED:
                        return "Failed to write file";
                    case IOCode::INVALID_FORMAT:
                        return "Invalid file format";
                    case IOCode::PERMISSION_DENIED:
                        return "Permission denied";
                    case IOCode::PATH_NOT_FOUND:
                        return "Path not found";
                    default:
                        return "Unknown IO error";
                }
            }
        };

        // 算法错误类别
        class AlgoErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "pcdl::algorithm";
            }

            std::string message(int ev) const override {
                switch (static_cast<AlgoCode>(ev)) {
                    case AlgoCode::Ok:
                        return "Success";
                    case AlgoCode::EMPTY_POINT_CLOUD:
                        return "Empty point cloud";
                    case AlgoCode::INVALID_PARAMETER:
                        return "Invalid parameter";
                    case AlgoCode::SEGMENTATION_FAILED:
                        return "Segmentation failed";
                    case AlgoCode::CLUSTERING_FAILED:
                        return "Clustering failed";
                    case AlgoCode::FITTING_FAILED:
                        return "Fitting failed";
                    case AlgoCode::CONVERGENCE_FAILED:
                        return "Convergence failed";
                    case AlgoCode::INSUFFICIENT_POINTS:
                        return "Insufficient points for operation";
                    case AlgoCode::DIMENSION_MISMATCH:
                        return "Dimension mismatch";
                    default:
                        return "Unknown algorithm error";
                }
            }
        };

        // 可视化错误类别
        class VisualizationErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "pcdl::visualization";
            }

            std::string message(int ev) const override {
                switch (static_cast<VisualizationCode>(ev)) {
                    case VisualizationCode::Ok:
                        return "Success";
                    case VisualizationCode::INIT_FAILED:
                        return "Visualization initialization failed";
                    case VisualizationCode::RENDER_FAILED:
                        return "Rendering failed";
                    case VisualizationCode::WINDOW_CREATE_FAILED:
                        return "Window creation failed";
                    case VisualizationCode::INVALID_COLOR_MODE:
                        return "Invalid color mode";
                    default:
                        return "Unknown visualization error";
                }
            }
        };

        // 通用错误类别
        class CommonErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "pcdl::common";
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
                    case CommonCode::INVALID_STATE:
                        return "Invalid state";
                    case CommonCode::TIMEOUT:
                        return "Operation timeout";
                    case CommonCode::CANCELLED:
                        return "Operation cancelled";
                    default:
                        return "Unknown common error";
                }
            }
        };

        // ============================================================================
        // Category 单例访问函数 - Category Singleton Accessors
        // ============================================================================

        inline const IOErrorCategory& io_category() {
            static IOErrorCategory instance;
            return instance;
        }

        inline const AlgoErrorCategory& algo_category() {
            static AlgoErrorCategory instance;
            return instance;
        }

        inline const VisualizationErrorCategory& vis_category() {
            static VisualizationErrorCategory instance;
            return instance;
        }

        inline const CommonErrorCategory& common_category() {
            static CommonErrorCategory instance;
            return instance;
        }

        // ============================================================================
        // make_error_code 函数 - Error Code Creation Functions
        // ============================================================================

        inline std::error_code make_error_code(IOCode e) {
            return {static_cast<int>(e), io_category()};
        }

        inline std::error_code make_error_code(AlgoCode e) {
            return {static_cast<int>(e), algo_category()};
        }

        inline std::error_code make_error_code(VisualizationCode e) {
            return {static_cast<int>(e), vis_category()};
        }

        inline std::error_code make_error_code(CommonCode e) {
            return {static_cast<int>(e), common_category()};
        }

        // ============================================================================
        // 辅助函数和类 - Helper Functions and Classes
        // ============================================================================

        // 错误结果包装类 - 用于返回值和错误码
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

        // void 特化
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

        // 异常类 - 基于 std::system_error
        class PCDLException : public std::system_error {
        public:
            template<typename ErrorEnum>
            explicit PCDLException(ErrorEnum e, const std::string& what_arg = "")
                : std::system_error(make_error_code(e), what_arg) {}

            PCDLException(std::error_code ec, const std::string& what_arg = "")
                : std::system_error(ec, what_arg) {}
        };

        // 便捷的检查宏
        #define PCDL_CHECK(condition, error_code) \
            if (!(condition)) { \
                throw pcdl::error_code::PCDLException(error_code, \
                    std::string(__FILE__) + ":" + std::to_string(__LINE__)); \
            }

        #define PCDL_RETURN_IF_ERROR(expr) \
            do { \
                auto _result = (expr); \
                if (!_result.ok()) { \
                    return _result.error(); \
                } \
            } while(0)

    } // namespace error_code
} // namespace pcdl

// ============================================================================
// std::error_code 特化 - Enable implicit conversion
// ============================================================================
namespace std {
    template<>
    struct is_error_code_enum<pcdl::error_code::IOCode> : true_type {};

    template<>
    struct is_error_code_enum<pcdl::error_code::AlgoCode> : true_type {};

    template<>
    struct is_error_code_enum<pcdl::error_code::VisualizationCode> : true_type {};

    template<>
    struct is_error_code_enum<pcdl::error_code::CommonCode> : true_type {};
}

#endif //POINTCLOUDDOCKLIB_PCDL_ERROR_CODE_H