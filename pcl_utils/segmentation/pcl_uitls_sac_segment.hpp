//
// Created by Mengfanyong on 2025/12/8.
//

#ifndef POINTCLOUDDOCKLIB_SAC_SEGMENT_HPP
#define POINTCLOUDDOCKLIB_SAC_SEGMENT_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

/**
 * @brief 判断SAC模型是否需要法线信息
 */
inline bool sacModelRequiresNormals(pcl::SacModel model) {
    switch (model) {
        case pcl::SACMODEL_CYLINDER:
        case pcl::SACMODEL_CONE:
        case pcl::SACMODEL_NORMAL_PLANE:
        case pcl::SACMODEL_NORMAL_SPHERE:
        case pcl::SACMODEL_NORMAL_PARALLEL_PLANE:
            return true;
        default:
            return false;
    }
}

/**
 * @brief 通用RANSAC拟合与分割函数，自动处理需要法线的模型
 *
 * @param cloud              输入点云
 * @param sac_model          SAC模型类型
 * @param distance_threshold 距离阈值
 * @param max_iterations     最大迭代次数
 * @param inlier_cloud       输出：内点点云
 * @param remaining_cloud    输出：剩余点云（可为nullptr）
 * @param coefficients       输出：模型系数
 * @param normal_search_radius 法线估计搜索半径（仅对需要法线的模型有效）
 * @param axis               轴向约束（可选，用于圆柱/圆锥）
 * @param axis_epsilon       轴向角度容差（弧度）
 * @param radius_limits      半径限制 [min, max]（可选，用于圆柱/圆锥/球）
 * @return true 成功, false 失败
 */
template <typename PointT>
bool fitAndSegment(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
    pcl::SacModel sac_model,
    float distance_threshold,
    int max_iterations,
    const typename pcl::PointCloud<PointT>::Ptr& inlier_cloud,
    const typename pcl::PointCloud<PointT>::Ptr& remaining_cloud,
    const pcl::ModelCoefficients::Ptr& coefficients,
    // 以下为可选参数
    float normal_search_radius = 0.03f,
    const Eigen::Vector3f& axis = Eigen::Vector3f::Zero(),
    float axis_epsilon = 0.1f,
    const std::pair<float, float>& radius_limits = {0.0f, 0.0f}
)
{
    if (!cloud || cloud->empty()) {
        return false;
    }
    if (!inlier_cloud || !coefficients) {
        return false;
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    bool needs_normals = sacModelRequiresNormals(sac_model);

    if (needs_normals) {
        // ========== 需要法线的模型（圆柱、圆锥等）==========

        // 1. 计算法线
        typename pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        typename pcl::NormalEstimation<PointT, pcl::Normal> ne;
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud);
        ne.setRadiusSearch(normal_search_radius);
        ne.compute(*normals);

        // 2. 使用带法线的分割器
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(sac_model);
        seg.setDistanceThreshold(distance_threshold);
        seg.setMaxIterations(max_iterations);

        // 法线距离权重
        seg.setNormalDistanceWeight(0.1);

        // 设置轴向约束（如果提供）
        if (!axis.isZero()) {
            seg.setAxis(axis);
            seg.setEpsAngle(axis_epsilon);
        }

        // 设置半径限制（如果提供）
        if (radius_limits.first > 0 || radius_limits.second > 0) {
            seg.setRadiusLimits(radius_limits.first, radius_limits.second);
        }

        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);
        seg.segment(*inliers, *coefficients);

    } else {
        // ========== 不需要法线的模型（平面、球、直线等）==========

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(sac_model);
        seg.setDistanceThreshold(distance_threshold);
        seg.setMaxIterations(max_iterations);

        // 设置轴向约束（如果提供，用于平行平面等）
        if (!axis.isZero()) {
            seg.setAxis(axis);
            seg.setEpsAngle(axis_epsilon);
        }

        // 设置半径限制（用于球、圆等）
        if (radius_limits.first > 0 || radius_limits.second > 0) {
            seg.setRadiusLimits(radius_limits.first, radius_limits.second);
        }

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    }

    // 检查是否找到模型
    if (inliers->indices.empty()) {
        return false;
    }

    // 提取内点和剩余点云
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    // 内点
    extract.setNegative(false);
    extract.filter(*inlier_cloud);

    // 剩余点
    if (remaining_cloud != nullptr) {
        extract.setNegative(true);
        extract.filter(*remaining_cloud);
    }

    return true;
}
#endif //POINTCLOUDDOCKLIB_SAC_SEGMENT_HPP