//
// PCDL Error Code Library - Usage Examples
// 展示如何在不同场景下使用错误库
//

#include "../../pcl_utils/error_code/pcdl_error_code.hpp"
#include <iostream>
#include <fstream>
#include <vector>

using namespace pcdl::error_code;

// ============================================================================
// 示例 1: 基本的错误码使用
// ============================================================================
void example1_basic_error_code() {
    std::cout << "\n=== Example 1: Basic Error Code Usage ===\n";

    // 创建错误码
    std::error_code ec = make_error_code(IOCode::FILE_NOT_FOUND);

    // 检查错误
    if (ec) {
        std::cerr << "Error occurred: " << ec.message() << "\n";
        std::cerr << "Error category: " << ec.category().name() << "\n";
        std::cerr << "Error value: 0x" << std::hex << ec.value() << std::dec << "\n";
    }

    // 创建成功的错误码
    std::error_code ok = make_error_code(IOCode::Ok);
    if (!ok) {
        std::cout << "Operation successful!\n";
    }
}

// ============================================================================
// 示例 2: 使用 Result 类返回值或错误
// ============================================================================
Result<std::vector<double>> readDataFromFile(const std::string& filename) {
    // 模拟文件读取
    std::ifstream file(filename);
    if (!file.is_open()) {
        // 返回错误
        return IOCode::FILE_NOT_FOUND;
    }

    std::vector<double> data;
    double value;
    while (file >> value) {
        data.push_back(value);
    }

    if (data.empty()) {
        // 返回算法错误
        return AlgoCode::EMPTY_POINT_CLOUD;
    }

    // 返回成功结果
    return data;
}

void example2_result_pattern() {
    std::cout << "\n=== Example 2: Result Pattern ===\n";

    // 尝试读取不存在的文件
    auto result = readDataFromFile("nonexistent.txt");

    if (result.ok()) {
        std::cout << "Read " << result.value().size() << " points\n";
    } else {
        std::cerr << "Failed to read file: " << result.error().message() << "\n";
    }

    // 使用 value_or 提供默认值
    auto data = result.value_or(std::vector<double>{1.0, 2.0, 3.0});
    std::cout << "Using " << data.size() << " points (with fallback)\n";
}

// ============================================================================
// 示例 3: 使用异常处理
// ============================================================================
void processPointCloud(const std::vector<double>& points) {
    // 参数验证
    if (points.empty()) {
        throw PCDLException(AlgoCode::EMPTY_POINT_CLOUD, "Point cloud is empty");
    }

    if (points.size() < 3) {
        throw PCDLException(AlgoCode::INSUFFICIENT_POINTS,
                           "Need at least 3 points, got " + std::to_string(points.size()));
    }

    std::cout << "Processing " << points.size() << " points...\n";
}

void example3_exception_handling() {
    std::cout << "\n=== Example 3: Exception Handling ===\n";

    try {
        std::vector<double> emptyCloud;
        processPointCloud(emptyCloud);
    } catch (const PCDLException& e) {
        std::cerr << "PCDL Exception: " << e.what() << "\n";
        std::cerr << "Error code: 0x" << std::hex << e.code().value() << std::dec << "\n";
        std::cerr << "Category: " << e.code().category().name() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << "\n";
    }
}

// ============================================================================
// 示例 4: 使用宏进行参数验证
// ============================================================================
Result<double> calculateMean(const std::vector<double>& data) {
    // 使用宏进行检查
    try {
        PCDL_CHECK(!data.empty(), AlgoCode::EMPTY_POINT_CLOUD);

        double sum = 0.0;
        for (double val : data) {
            sum += val;
        }

        return sum / data.size();
    } catch (const PCDLException& e) {
        return e.code();
    }
}

void example4_macro_usage() {
    std::cout << "\n=== Example 4: Macro Usage ===\n";

    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    auto result = calculateMean(data);

    if (result) {
        std::cout << "Mean: " << result.value() << "\n";
    } else {
        std::cerr << "Error: " << result.error().message() << "\n";
    }

    // 测试空数据
    std::vector<double> emptyData;
    auto emptyResult = calculateMean(emptyData);
    if (!emptyResult) {
        std::cerr << "Expected error: " << emptyResult.error().message() << "\n";
    }
}

// ============================================================================
// 示例 5: 跨模块错误传播
// ============================================================================
Result<void> saveVisualization(const std::string& filename) {
    // 可视化初始化
    bool initSuccess = true; // 模拟初始化
    if (!initSuccess) {
        return VisualizationCode::INIT_FAILED;
    }

    // 渲染
    bool renderSuccess = true;
    if (!renderSuccess) {
        return VisualizationCode::RENDER_FAILED;
    }

    // 保存文件
    std::ofstream file(filename);
    if (!file.is_open()) {
        return IOCode::FILE_WRITE_FAILED;
    }

    // 成功
    return Result<void>();
}

void example5_error_propagation() {
    std::cout << "\n=== Example 5: Error Propagation ===\n";

    auto result = saveVisualization("output.png");

    if (result.ok()) {
        std::cout << "Visualization saved successfully\n";
    } else {
        const auto& ec = result.error();
        std::cerr << "Failed to save visualization: " << ec.message() << "\n";
        std::cerr << "Error from module: " << ec.category().name() << "\n";
    }
}

// ============================================================================
// 示例 6: 不同错误类别的比较
// ============================================================================
void example6_error_comparison() {
    std::cout << "\n=== Example 6: Error Category Comparison ===\n";

    std::error_code io_error = make_error_code(IOCode::FILE_NOT_FOUND);
    std::error_code algo_error = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);
    std::error_code vis_error = make_error_code(VisualizationCode::RENDER_FAILED);

    // 比较错误类别
    if (io_error.category() == io_category()) {
        std::cout << "This is an IO error: " << io_error.message() << "\n";
    }

    if (algo_error.category() == algo_category()) {
        std::cout << "This is an algorithm error: " << algo_error.message() << "\n";
    }

    if (vis_error.category() == vis_category()) {
        std::cout << "This is a visualization error: " << vis_error.message() << "\n";
    }

    // 不同类别的错误不相等
    if (io_error != algo_error) {
        std::cout << "IO error and Algorithm error are different\n";
    }
}

// ============================================================================
// 示例 7: 自定义错误处理策略
// ============================================================================
class ErrorHandler {
public:
    template<typename T>
    static T handleResult(Result<T> result, const T& fallback) {
        if (result.ok()) {
            return result.value();
        }

        // 记录错误
        logError(result.error());

        // 返回备用值
        return fallback;
    }

private:
    static void logError(const std::error_code& ec) {
        std::cerr << "[ERROR] " << ec.category().name()
                  << ": " << ec.message()
                  << " (code: 0x" << std::hex << ec.value() << std::dec << ")\n";
    }
};

void example7_custom_error_handling() {
    std::cout << "\n=== Example 7: Custom Error Handling Strategy ===\n";

    auto result = readDataFromFile("missing.txt");

    // 使用自定义错误处理器
    auto data = ErrorHandler::handleResult(result, std::vector<double>{0.0, 0.0, 0.0});

    std::cout << "Got " << data.size() << " points (with custom handler)\n";
}

// ============================================================================
// 示例 8: 链式调用和错误传播
// ============================================================================
Result<void> pipeline(const std::string& inputFile, const std::string& outputFile) {
    // 读取数据
    auto readResult = readDataFromFile(inputFile);
    if (!readResult.ok()) {
        return readResult.error();
    }

    // 计算均值
    auto meanResult = calculateMean(readResult.value());
    if (!meanResult.ok()) {
        return meanResult.error();
    }

    std::cout << "Pipeline - Mean value: " << meanResult.value() << "\n";

    // 保存结果
    return saveVisualization(outputFile);
}

void example8_pipeline() {
    std::cout << "\n=== Example 8: Pipeline with Error Propagation ===\n";

    auto result = pipeline("input.txt", "output.png");

    if (result.ok()) {
        std::cout << "Pipeline completed successfully\n";
    } else {
        std::cerr << "Pipeline failed: " << result.error().message() << "\n";
    }
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "========================================\n";
    std::cout << "PCDL Error Code Library - Usage Examples\n";
    std::cout << "========================================\n";

    try {
        example1_basic_error_code();
        example2_result_pattern();
        example3_exception_handling();
        example4_macro_usage();
        example5_error_propagation();
        example6_error_comparison();
        example7_custom_error_handling();
        example8_pipeline();

    } catch (const std::exception& e) {
        std::cerr << "\nUnexpected error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n========================================\n";
    std::cout << "All examples completed!\n";
    std::cout << "========================================\n";

    return 0;
}

