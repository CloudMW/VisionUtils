//
// Created by Mengfanyong on 2025/12/8.
//

#ifndef POINTCLOUDDOCKLIB_PCDL_CLUSTERS_HPP
#define POINTCLOUDDOCKLIB_PCDL_CLUSTERS_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


namespace pcdl::segmentation
{
    template <typename PointT>
    bool euclideanClustering(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                             std::vector<typename pcl::PointCloud<PointT>::Ptr>& clusters,
                             const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudRGB,
                             float cluster_tolerance,
                             int min_cluster_size,
                             int max_cluster_size)
    {
        if (!cloud || cloud->empty()) return false;

        clusters.clear();

        // 构建 kd-tree
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
        tree->setInputCloud(cloud);

        // 设置聚类器
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        clusters.reserve(cluster_indices.size());
        for (const auto& indices : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
            cluster_cloud->reserve(indices.indices.size());
            for (int idx : indices.indices)
            {
                cluster_cloud->push_back((*cloud)[idx]);
            }
            cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->points.size());
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;
            clusters.push_back(cluster_cloud);
        }

        if (cloudRGB != nullptr)
        {
            // 处理 RGB 点云的聚类结果（如果需要）
            std::mt19937 gen(std::random_device{}());
            std::uniform_int_distribution<int> dist(0, 255);

            // 这里可以根据需要将 RGB 信息与 clusters 关联起来
            for (size_t i = 0; i < clusters.size(); ++i)
            {
                uint8_t r = static_cast<uint8_t>(dist(gen));
                uint8_t g = static_cast<uint8_t>(dist(gen));
                uint8_t b = static_cast<uint8_t>(dist(gen));


                for (const auto& point : clusters[i]->points)
                {
                    pcl::PointXYZRGB rgb_point;
                    rgb_point.x = point.x;
                    rgb_point.y = point.y;
                    rgb_point.z = point.z;
                    rgb_point.r = r;
                    rgb_point.g = g;
                    rgb_point.b = b;
                    // 将 rgb_point 存储或处理
                    cloudRGB->push_back(rgb_point);
                }
            }
        }
        return true;
    }
}

#endif //POINTCLOUDDOCKLIB_PCDL_CLUSTERS_HPP
