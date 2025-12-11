//
// 简单的单元测试 - 验证错误库功能
//

#include "../../pcl_utils/error_code/pcdl_error_code.hpp"
#include <iostream>
#include <cassert>

using namespace pcdl::error_code;

// 测试计数器
int passed = 0;
int failed = 0;

#define TEST(name) \
    void test_##name(); \
    void run_test_##name() { \
        std::cout << "Running: " << #name << "... "; \
        try { \
            test_##name(); \
            std::cout << "PASSED\n"; \
            passed++; \
        } catch (const std::exception& e) { \
            std::cout << "FAILED: " << e.what() << "\n"; \
            failed++; \
        } \
    } \
    void test_##name()

// ============================================================================
// 测试用例
// ============================================================================

TEST(error_code_creation) {
    auto ec1 = make_error_code(IOCode::FILE_NOT_FOUND);
    assert(ec1);
    assert(ec1.value() == 0x800101);

    auto ec2 = make_error_code(IOCode::Ok);
    assert(!ec2);

    auto ec3 = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);
    assert(ec3);
    assert(ec3.value() == 0x800201);
}

TEST(error_code_message) {
    auto ec = make_error_code(IOCode::FILE_NOT_FOUND);
    std::string msg = ec.message();
    assert(msg == "File not found");

    auto ec2 = make_error_code(AlgoCode::INVALID_PARAMETER);
    assert(ec2.message() == "Invalid parameter");
}

TEST(error_category) {
    auto ec1 = make_error_code(IOCode::FILE_NOT_FOUND);
    auto ec2 = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);

    assert(ec1.category() == io_category());
    assert(ec2.category() == algo_category());
    assert(ec1.category() != ec2.category());

    assert(std::string(ec1.category().name()) == "pcdl::io");
    assert(std::string(ec2.category().name()) == "pcdl::algorithm");
}

TEST(result_success) {
    Result<int> result(42);

    assert(result.ok());
    assert(result);
    assert(result.value() == 42);
    assert(result.value_or(0) == 42);
}

TEST(result_error) {
    Result<int> result(IOCode::FILE_NOT_FOUND);

    assert(!result.ok());
    assert(!result);
    assert(result.error());
    assert(result.value_or(100) == 100);
}

TEST(result_void_success) {
    Result<void> result;

    assert(result.ok());
    assert(result);
    assert(!result.error());
}

TEST(result_void_error) {
    Result<void> result(AlgoCode::SEGMENTATION_FAILED);

    assert(!result.ok());
    assert(!result);
    assert(result.error());
    assert(result.error().value() == 0x800203);
}

TEST(result_from_enum) {
    Result<double> result(AlgoCode::CONVERGENCE_FAILED);

    assert(!result.ok());
    assert(result.error().category() == algo_category());
}

TEST(exception_basic) {
    try {
        throw PCDLException(IOCode::FILE_NOT_FOUND, "test.txt");
        assert(false);  // 不应该到达这里
    } catch (const PCDLException& e) {
        assert(e.code().value() == 0x800101);
        std::string what = e.what();
        assert(what.find("File not found") != std::string::npos);
    }
}

TEST(exception_from_error_code) {
    std::error_code ec = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);

    try {
        throw PCDLException(ec, "No data");
        assert(false);
    } catch (const PCDLException& e) {
        assert(e.code() == ec);
    }
}

TEST(multiple_categories) {
    auto io = make_error_code(IOCode::FILE_NOT_FOUND);
    auto algo = make_error_code(AlgoCode::EMPTY_POINT_CLOUD);
    auto vis = make_error_code(VisualizationCode::RENDER_FAILED);
    auto common = make_error_code(CommonCode::TIMEOUT);

    assert(io.category() == io_category());
    assert(algo.category() == algo_category());
    assert(vis.category() == vis_category());
    assert(common.category() == common_category());

    // 所有类别互不相同
    assert(io.category() != algo.category());
    assert(io.category() != vis.category());
    assert(algo.category() != vis.category());
}

TEST(error_comparison) {
    auto ec1 = make_error_code(IOCode::FILE_NOT_FOUND);
    auto ec2 = make_error_code(IOCode::FILE_NOT_FOUND);
    auto ec3 = make_error_code(IOCode::FILE_OPEN_FAILED);

    assert(ec1 == ec2);
    assert(ec1 != ec3);
}

TEST(implicit_conversion) {
    // 测试隐式转换到std::error_code
    std::error_code ec = IOCode::FILE_NOT_FOUND;
    assert(ec);
    assert(ec.value() == 0x800101);
}

// 辅助函数用于测试
Result<int> divide(int a, int b) {
    if (b == 0) {
        return AlgoCode::INVALID_PARAMETER;
    }
    return a / b;
}

TEST(real_world_result) {
    auto result1 = divide(10, 2);
    assert(result1.ok());
    assert(result1.value() == 5);

    auto result2 = divide(10, 0);
    assert(!result2.ok());
    assert(result2.error().category() == algo_category());
}

Result<void> validateData(int size) {
    if (size <= 0) {
        return AlgoCode::EMPTY_POINT_CLOUD;
    }
    if (size < 3) {
        return AlgoCode::INSUFFICIENT_POINTS;
    }
    return Result<void>();
}

TEST(real_world_void_result) {
    auto result1 = validateData(10);
    assert(result1.ok());

    auto result2 = validateData(0);
    assert(!result2.ok());
    assert(result2.error().value() == 0x800201);

    auto result3 = validateData(2);
    assert(!result3.ok());
    assert(result3.error().value() == 0x800207);
}

TEST(all_io_codes) {
    std::vector<IOCode> codes = {
        IOCode::Ok,
        IOCode::FILE_NOT_FOUND,
        IOCode::FILE_OPEN_FAILED,
        IOCode::FILE_READ_FAILED,
        IOCode::FILE_WRITE_FAILED,
        IOCode::INVALID_FORMAT,
        IOCode::PERMISSION_DENIED,
        IOCode::PATH_NOT_FOUND
    };

    for (auto code : codes) {
        auto ec = make_error_code(code);
        assert(!ec.message().empty());
    }
}

TEST(all_algo_codes) {
    std::vector<AlgoCode> codes = {
        AlgoCode::Ok,
        AlgoCode::EMPTY_POINT_CLOUD,
        AlgoCode::INVALID_PARAMETER,
        AlgoCode::SEGMENTATION_FAILED,
        AlgoCode::CLUSTERING_FAILED,
        AlgoCode::FITTING_FAILED,
        AlgoCode::CONVERGENCE_FAILED,
        AlgoCode::INSUFFICIENT_POINTS,
        AlgoCode::DIMENSION_MISMATCH
    };

    for (auto code : codes) {
        auto ec = make_error_code(code);
        assert(!ec.message().empty());
    }
}

TEST(result_move_semantics) {
    std::vector<int> large_data(1000, 42);
    Result<std::vector<int>> result(std::move(large_data));

    assert(result.ok());
    assert(result.value().size() == 1000);
    assert(large_data.empty());  // 已被移动
}

// ============================================================================
// 主测试运行器
// ============================================================================

int main() {
    std::cout << "========================================\n";
    std::cout << "PCDL Error Code Library - Unit Tests\n";
    std::cout << "========================================\n\n";

    // 运行所有测试
    run_test_error_code_creation();
    run_test_error_code_message();
    run_test_error_category();
    run_test_result_success();
    run_test_result_error();
    run_test_result_void_success();
    run_test_result_void_error();
    run_test_result_from_enum();
    run_test_exception_basic();
    run_test_exception_from_error_code();
    run_test_multiple_categories();
    run_test_error_comparison();
    run_test_implicit_conversion();
    run_test_real_world_result();
    run_test_real_world_void_result();
    run_test_all_io_codes();
    run_test_all_algo_codes();
    run_test_result_move_semantics();

    // 打印总结
    std::cout << "\n========================================\n";
    std::cout << "Test Summary:\n";
    std::cout << "  Passed: " << passed << "\n";
    std::cout << "  Failed: " << failed << "\n";
    std::cout << "  Total:  " << (passed + failed) << "\n";
    std::cout << "========================================\n";

    return failed > 0 ? 1 : 0;
}

