//
// Created by mfy on 2025/12/2.
//

#ifndef POINTCLOUDDOCKLIB_TXT_IO_H
#define POINTCLOUDDOCKLIB_TXT_IO_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <spdlog/spdlog.h>
#include <iosfwd>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#ifdef _WIN32
#include <direct.h>
#endif
namespace pcl_utils
{
    namespace io
    {
        // ---- 生成当前时间：YYYYMMDD_HHMMSS ----
        inline std::string getCurrentTimeString()
        {
            auto now = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            std::tm tm;
#ifdef _WIN32
            localtime_s(&tm, &t);
#else
            localtime_r(&t, &tm);
#endif

            std::stringstream ss;
            ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
            return ss.str();
        }

        // ---- 检查目录是否存在 ----
        inline bool directoryExists(const std::string& path)
        {
            struct stat info;
            return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
        }

        // ---- 创建目录（支持嵌套路径）----
        inline bool createDirectoryRecursive(const std::string& path)
        {
            if (path.empty()) return false;
            if (directoryExists(path)) return true;

            std::string tmp;
            for (size_t i = 0; i < path.size(); ++i)
            {
                tmp += path[i];

                if (path[i] == '/' || path[i] == '\\')
                {
                    if (!tmp.empty() && !directoryExists(tmp))
                    {
#ifdef _WIN32
                        if (_mkdir(tmp.c_str()) != 0) return false;
#else
                        if (mkdir(tmp.c_str(), 0755) != 0) return false;
#endif
                    }
                }
            }

            // 创建最后一级
            if (!directoryExists(tmp))
            {
#ifdef _WIN32
                if (_mkdir(tmp.c_str()) != 0) return false;
#else
                if (mkdir(tmp.c_str(), 0755) != 0) return false;
#endif
            }
            return true;
        }

        // ---- 保存点云为 TXT ----
        template <typename PointT>
        bool savePointCloudTXT(const typename pcl::PointCloud<PointT>::ConstPtr& input_pcd,
                               const std::string& file_path)
        {
            // 创建目录（递归）
            if (!createDirectoryRecursive(file_path))
            {
                std::cerr << "Failed to create directory: " << file_path << std::endl;
                return false;
            }

#ifdef _WIN32
            std::string filename = file_path + "\\" + getCurrentTimeString() + ".txt";
#else
            std::string filename = file_path + "/" + getCurrentTimeString() + ".txt";
#endif

            std::ofstream file(filename);
            if (!file.is_open())
            {
                std::cerr << "Cannot open file: " << filename << std::endl;
                return false;
            }

            for (const auto& p : input_pcd->points)
            {
                file << p.x << " " << p.y << " " << p.z << "\n";
            }

            file.close();
            spdlog::info("Saved TXT point cloud: {}", filename);
            return true;
        }


        bool readTXTToPCLXYZI(const std::string& file_path, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
        {
            // 初始化点云
            cloud->clear();

            // 打开文件
            std::ifstream file(file_path);
            if (!file.is_open())
            {
                spdlog::error("can't open:{}", file_path);
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
                if (!(ss >> p.x >> p.y >> p.z >> p.intensity))
                {
                    spdlog::warn("line {} format error ,skipp:{}", line_num, line);
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
                spdlog::error("data is empty, file:{} ", file_path);
                return false;
            }
            spdlog::info("open data success!file :{},point size:{}", file_path, cloud->size());
            return true;
        }

        bool readTXTToPCLXYZ(const std::string& file_path, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
        {
            // 初始化点云
            cloud->clear();

            // 打开文件
            std::ifstream file(file_path);
            if (!file.is_open())
            {
                spdlog::error("can't open:{}", file_path);
                return false;
            }

            std::string line;
            int line_num = 0;

            // 逐行解析
            while (std::getline(file, line))
            {
                line_num++;
                std::stringstream ss(line);
                pcl::PointXYZ p;

                // 解析 x y z（忽略多余空格/制表符）
                if (!(ss >> p.x >> p.y >> p.z))
                {
                    spdlog::warn("line {} format error ,skipp:{}", line_num, line);
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
                spdlog::error("data is empty, file:{} ", file_path);
                return false;
            }
            spdlog::info("open data success!file :{},point size:{}", file_path, cloud->size());
            return true;
        }
    } // io
} // pcdl

#endif //POINTCLOUDDOCKLIB_TXT_IO_H
