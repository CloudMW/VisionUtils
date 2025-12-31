//
// Created by mfy20 on 2025/12/20.
//

#ifndef VISIONUTILS_PCL_UTILS_CLOUD_SAFETY_HPP
#define VISIONUTILS_PCL_UTILS_CLOUD_SAFETY_HPP

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <string>
#include <Eigen/Eigen>
#include <spdlog/spdlog.h>


namespace pcl_utils {
    /*!
 * @code
 * cloud_safety::check<PointT>(cloud, 0);
 * @endcode
 */
    namespace cloud_safety {
        // ========================
        // 点云状态枚举
        // ========================
        enum class CloudStatus {
            OK = 0,
            NullPtr,
            Empty,
            TooSmall,
            ContainsInvalid
        };

        // ========================
        // 状态转字符串（调试 / 日志）
        // ========================
        inline const char *toString(CloudStatus status) {
            switch (status) {
                case CloudStatus::OK: return "OK";
                case CloudStatus::NullPtr: return "Null pointer";
                case CloudStatus::Empty: return "Empty cloud";
                case CloudStatus::TooSmall: return "Too few points";
                default: return "Unknown";
            }
        }

        // ========================
        // 核心安全检查（ConstPtr）
        // ========================
        template<typename PointT>
        CloudStatus check(
            const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
            std::size_t min_points = 1) {
            if (!cloud)
                return CloudStatus::NullPtr;

            if (cloud->empty())
                return CloudStatus::Empty;

            if (cloud->size() < min_points)
                return CloudStatus::TooSmall;

            return CloudStatus::OK;
        }

        // ========================
        // 重载：const 引用版本
        // （用于算法内部）
        // ========================
        template<typename PointT>
        CloudStatus check(
            const pcl::PointCloud<PointT> &cloud,
            std::size_t min_points = 1) {
            if (cloud.empty())
                return CloudStatus::Empty;

            if (cloud.size() < min_points)
                return CloudStatus::TooSmall;


            return CloudStatus::OK;
        }

        // ========================
        // 统一错误输出（可选）
        // ========================
        inline void printError(
            const std::string &tag,
            CloudStatus status) {
            if (status != CloudStatus::OK) {
                spdlog::info("[{}]  Cloud check failed: {}", tag, toString(status));
            }
        }
    } // namespace cloud_safety
}
#endif //VISIONUTILS_PCL_UTILS_CLOUD_SAFETY_HPP
