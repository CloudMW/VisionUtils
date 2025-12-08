//
// Created by Mengfanyong on 2025/12/4.
//

#ifndef POINTCLOUDDOCKLIB_DEBUG_HPP
#define POINTCLOUDDOCKLIB_DEBUG_HPP
#include <functional>

/**
 * @brief Debug等级定义
 * NONE    - 不输出任何debug信息
 * RESULT  - 只输出最终结果
 * PROCESS - 输出过程信息（包含RESULT）
 * VERBOSE - 输出详细信息（包含PROCESS和RESULT）
 */
enum class DebugLevel {
    NONE = 0,      // 不输出
    RESULT = 1,    // 结果等级
    PROCESS = 2,   // 过程等级
    VERBOSE = 3,    // 详细等级
    POINTSAVE = 4   // 保存点云等级
};

/**
 * @brief Debug配置管理类
 * 用于全局设置和查询debug等级
 */
class DebugConfig {
private:
    static DebugLevel vis_global_level;
    static DebugLevel log_global_level;

public:
    /**
     * @brief 设置全局debug等级
     * @param level 要设置的debug等级
     */
    static void setVisDebugLevel(DebugLevel level) {
        vis_global_level = level;
    }
    static void setLogDebugLevel(DebugLevel level) {
        log_global_level = level;
    }



    /**
     * @brief 判断是否应该执行某个等级的debug代码
     * @param required_level 所需的debug等级
     * @return 如果当前全局等级 >= 所需等级，返回true
     */
    static bool shouldExecuteVis(DebugLevel required_level) {
        return static_cast<int>(vis_global_level) >= static_cast<int>(required_level);
    }
    static bool shouldExecuteLog(DebugLevel required_level) {
        return static_cast<int>(log_global_level) >= static_cast<int>(required_level);
    }
};

// 初始化静态成员变量（默认为NONE）
DebugLevel DebugConfig::vis_global_level = DebugLevel::NONE;
DebugLevel DebugConfig::log_global_level = DebugLevel::NONE;

/**
 * @brief 根据debug等级执行函数
 * @tparam Func 可调用对象类型
 * @param required_level 执行该函数所需的debug等级
 * @param f 要执行的函数或lambda表达式
 *
 * 使用示例：
 * // 设置全局debug等级
 * DebugConfig::setDebugLevel(DebugLevel::PROCESS);
 *
 * // 详细信息（只在VERBOSE等级才输出）
 * debug_exec(DebugLevel::VERBOSE, []() {
 *     spdlog::info("详细信息：点云数量={}", cloud->size());
 * });
 *
 * // 过程信息（在PROCESS或VERBOSE等级输出）
 * debug_exec(DebugLevel::PROCESS, []() {
 *     spdlog::info("过程：开始聚类分析");
 * });
 *
 * // 结果信息（在RESULT、PROCESS或VERBOSE等级都会输出）
 * debug_exec(DebugLevel::RESULT, []() {
 *     spdlog::info("结果：找到 {} 个聚类", num_clusters);
 * });
 */
template<typename Func>
inline void debug_exec_vis(DebugLevel required_level, Func&& f) {
    if (DebugConfig::shouldExecuteVis(required_level)) {
        std::invoke(std::forward<Func>(f));
    }
}

template<typename Func>
inline void debug_exec_log(DebugLevel required_level, Func&& f) {
    if (DebugConfig::shouldExecuteLog(required_level)) {
        std::invoke(std::forward<Func>(f));
    }
}
#endif //POINTCLOUDDOCKLIB_DEBUG_HPP