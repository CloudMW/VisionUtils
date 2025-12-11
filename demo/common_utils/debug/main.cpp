//
// Created by Mengfanyong on 2025/12/4.
//

#include "pcdl/debug/pcdl_debug.hpp"
#include "spdlog/spdlog.h"
int main() {
    // 设置全局debug等级为PROCESS
    DebugConfig::setVisDebugLevel(DebugLevel::PROCESS);
    DebugConfig::setLogDebugLevel(DebugLevel::PROCESS);

    // 示例：根据不同等级执行不同的debug代码
    if (DebugConfig::shouldExecuteLog(DebugLevel::RESULT)) {
        spdlog::info("[RESULT] 这是结果等级的日志信息。");
    }

    if (DebugConfig::shouldExecuteLog(DebugLevel::PROCESS)) {
        spdlog::info("[PROCESS] 这是过程等级的日志信息。");
    }

    if (DebugConfig::shouldExecuteLog(DebugLevel::VERBOSE)) {
        spdlog::info("[VERBOSE] 这是详细等级的日志信息。");
    }

    if (DebugConfig::shouldExecuteLog(DebugLevel::POINTSAVE)) {
        spdlog::info("[POINTSAVE] 这是保存点云等级的日志信息。");
    }
    if (DebugConfig::shouldExecuteVis(DebugLevel::RESULT)) {
        spdlog::info("[RESULT] 这是结果等级的显示信息。");
    }

    return 0;
}