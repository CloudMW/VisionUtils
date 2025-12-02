//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_TXT_IO_H
#define POINTCLOUDDOCKLIB_TXT_IO_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace pcdl {
    namespace io {

            bool readTXTToPCLXYZI(const std::string& file_path, const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
;
    } // io
} // pcdl

#endif //POINTCLOUDDOCKLIB_TXT_IO_H