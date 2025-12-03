//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_TXT_IO_H
#define POINTCLOUDDOCKLIB_TXT_IO_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcdl_io_txt_export.h"

namespace pcdl {
    namespace io {

       PCDL_IO_TXT_API  bool readTXTToPCLXYZI(const std::string& file_path, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
;
    } // io
} // pcdl

#endif //POINTCLOUDDOCKLIB_TXT_IO_H