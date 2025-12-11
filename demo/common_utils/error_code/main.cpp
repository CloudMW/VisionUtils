//
// Created by mfy on 2025/12/2.
//

#include "pcdl/error_code/pcdl_error_code.hpp"
#include <iostream>
int main() {

    pcdl::error_code::AlgoCode code = pcdl::error_code::AlgoCode::FILE_NOT_FOUND;
    const char* message = pcdl::error_code::getErrorMessage(code);
    // 输出错误信息
    printf("Error Code: %d, Message: %s\n", static_cast<int>(code), message);
    std::cout<<std::hex<<static_cast<int>(code)<<std::endl;
    return 0;
}