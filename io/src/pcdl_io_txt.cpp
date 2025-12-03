//
// Created by mfy on 2025/12/2.
//

#include "pcdl_io_txt.h"

#include <string>
#include <spdlog/spdlog.h>
#include <fstream>

#define DLL_EXPORTS
namespace pcdl {
    namespace io {

        bool readTXTToPCLXYZI(const std::string& file_path, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
        {
            // 初始化点云
            cloud->clear();

            // 打开文件
            std::ifstream file(file_path);
            if (!file.is_open())
            {
                spdlog::error("无法打开文件：{}", file_path);
                return false;
            }

            std::string line;
            int line_num = 0;

            // 逐行解析
            while (std::getline(file, line))
            {
                line_num++;
                std::stringstream ss(line);
                pcl::PointXYZI p;

                // 解析 x y z（忽略多余空格/制表符）
                if (!(ss >> p.x >> p.y >> p.z>>p.intensity))
                {
                    spdlog::warn("第 {} 行格式错误，跳过：{}", line_num, line);
                    continue;
                }

                // 可选：忽略行内多余数据（如误加的颜色/强度值）
                // float dummy;
                // if (ss >> dummy) {
                //     std::cerr << "[WARNING] 第 " << line_num << " 行数据冗余，仅读取 x y z" << std::endl;
                // }
                cloud->push_back(p);
            }

            file.close();

            // 验证读取结果
            if (cloud->empty())
            {
                spdlog::error("点云数据为空，文件：{}", file_path);
                return false;
            }
            spdlog::info("点云读取成功！文件：{}，点数：{}", file_path, cloud->size());
            return true;
        }


    } // io
} // pcdl