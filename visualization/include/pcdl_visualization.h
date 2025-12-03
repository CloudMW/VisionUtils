//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#define POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
#include <pcl/visualization/pcl_visualizer.h>
#include "pcdl_visualization_export.h"


namespace pcdl
{
    namespace visualization
    {
        template <typename PointT>
         bool showPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud,
                                    const typename pcl::PointCloud<PointT>::ConstPtr& target_cloud,
                                    const std::string& window_name);
        /*template <typename PointT>
        DLL_API bool showPointCloud<pcl::PointXYZI>(
            const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
            const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
            const std::string&);*/
      extern template
      PCDL_VISUALIZATION_API    bool showPointCloud<pcl::PointXYZI>(
            const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
            const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&,
            const std::string&);
    }
}



#endif //POINTCLOUDDOCKLIB_PCDL_VISUALIZATION_H
