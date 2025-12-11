//
// Created by Mengfanyong on 2025/12/11.
//

#ifndef POINTCLOUDDOCKLIB_TIMER_HPP
#define POINTCLOUDDOCKLIB_TIMER_HPP


#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>

/**
 * @brief 算法计时器类 - 用于测量算法各阶段的执行时间
 *
 * 使用示例:
 *   AlgorithmTimer timer;
 *   timer.start("preprocessing");
 *   // ... 预处理代码 ...
 *   timer.stop("preprocessing");
 *
 *   timer.start("computation");
 *   // ... 计算代码 ...
 *   timer.stop("computation");
 *
 *   std::cout << timer.getSummary() << std::endl;
 */
class AlgorithmTimer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::milli>; // 毫秒

    /**
     * @brief 构造函数 - 自动记录整体开始时间
     */
    AlgorithmTimer() : total_start_(Clock::now()), is_running_(false) {}

    /**
     * @brief 开始计时某个阶段
     * @param stage_name 阶段名称
     */
    void start(const std::string& stage_name) {
        if (current_stage_start_.count(stage_name) > 0 &&
            current_stage_start_[stage_name] != TimePoint{}) {
            std::cerr << "Warning: Stage '" << stage_name
                      << "' is already running!" << std::endl;
            return;
        }
        current_stage_start_[stage_name] = Clock::now();
    }

    /**
     * @brief 停止计时某个阶段
     * @param stage_name 阶段名称
     */
    void stop(const std::string& stage_name) {
        auto end_time = Clock::now();

        if (current_stage_start_.count(stage_name) == 0 ||
            current_stage_start_[stage_name] == TimePoint{}) {
            std::cerr << "Warning: Stage '" << stage_name
                      << "' was not started!" << std::endl;
            return;
        }

        Duration elapsed = end_time - current_stage_start_[stage_name];
        stage_durations_[stage_name] += elapsed.count();
        current_stage_start_[stage_name] = TimePoint{}; // 重置
    }

    /**
     * @brief 获取某个阶段的总耗时(毫秒)
     * @param stage_name 阶段名称
     * @return 耗时(毫秒)
     */
    double getStageTime(const std::string& stage_name) const {
        auto it = stage_durations_.find(stage_name);
        return (it != stage_durations_.end()) ? it->second : 0.0;
    }

    /**
     * @brief 获取从构造函数开始到现在的总耗时(毫秒)
     * @return 总耗时(毫秒)
     */
    double getTotalTime() const {
        Duration elapsed = Clock::now() - total_start_;
        return elapsed.count();
    }

    /**
     * @brief 重置所有计时数据
     */
    void reset() {
        stage_durations_.clear();
        current_stage_start_.clear();
        total_start_ = Clock::now();
    }

    /**
     * @brief 获取所有阶段的名称列表
     */
    std::vector<std::string> getStageNames() const {
        std::vector<std::string> names;
        for (const auto& pair : stage_durations_) {
            names.push_back(pair.first);
        }
        return names;
    }

    /**
     * @brief 获取格式化的摘要报告
     * @param precision 小数点精度(默认2位)
     * @return 格式化的字符串报告
     */
    std::string getSummary(int precision = 2) const {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision);

        oss << "========== Algorithm Timer Summary ==========\n";

        double total = getTotalTime();
        oss << "Total Time: " << total << " ms\n";
        oss << "---------------------------------------------\n";

        if (stage_durations_.empty()) {
            oss << "No stages recorded.\n";
        } else {
            // 计算所有阶段时间总和
            double stages_sum = 0.0;
            for (const auto& pair : stage_durations_) {
                stages_sum += pair.second;
            }

            // 输出各阶段信息
            for (const auto& pair : stage_durations_) {
                double percentage = (total > 0) ? (pair.second / total * 100.0) : 0.0;
                oss << std::setw(20) << std::left << pair.first << ": "
                    << std::setw(10) << std::right << pair.second << " ms ("
                    << std::setw(5) << percentage << "%)\n";
            }

            // 如果有未计时的时间，显示出来
            if (stages_sum < total) {
                double other = total - stages_sum;
                double percentage = (other / total * 100.0);
                oss << std::setw(20) << std::left << "[Other/Overhead]" << ": "
                    << std::setw(10) << std::right << other << " ms ("
                    << std::setw(5) << percentage << "%)\n";
            }
        }

        oss << "=============================================";
        return oss.str();
    }

    /**
     * @brief 打印摘要报告到控制台
     */
    void printSummary(int precision = 2) const {
        std::cout << getSummary(precision) << std::endl;
    }

    /**
     * @brief 便捷的 RAII 风格计时器
     *
     * 使用示例:
     *   {
     *       AlgorithmTimer::ScopedTimer timer(main_timer, "stage1");
     *       // ... 代码 ...
     *   } // 自动调用stop
     */
    class ScopedTimer {
    public:
        ScopedTimer(AlgorithmTimer& timer, const std::string& stage_name)
            : timer_(timer), stage_name_(stage_name) {
            timer_.start(stage_name_);
        }

        ~ScopedTimer() {
            timer_.stop(stage_name_);
        }

        // 禁止拷贝
        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;

    private:
        AlgorithmTimer& timer_;
        std::string stage_name_;
    };

private:
    TimePoint total_start_;                                    // 总体开始时间
    std::unordered_map<std::string, double> stage_durations_;  // 各阶段累计时间
    std::unordered_map<std::string, TimePoint> current_stage_start_; // 当前运行阶段的开始时间
    bool is_running_;                                          // 是否有阶段正在运行
};


#endif //POINTCLOUDDOCKLIB_TIMER_HPP