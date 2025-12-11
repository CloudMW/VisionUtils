//
// Created by mfy on 2025/12/11.
//

#include <spdlog/spdlog.h>
#include "cv_utils/aruco/cv_utils_aruco.hpp"
#include "cv_utils/visualization/cv_utils_visualization.hpp"
using namespace cv_utils::visualization;
int main() {
    std::string data_path = std::string(DEMO_PATH) + "data/image_aruco.png";

    SPDLOG_INFO("data_path : {}",data_path);
    cv::Mat in_mat = cv::imread(data_path);
    if (in_mat.empty()) {
        std::cerr << "Failed to load image: " <<data_path  << "\n";
        return 1;
    }
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Mat out_mat;
    cv_utils::aruco::detectMarkes(in_mat,markerIds,markerCorners,out_mat);

    showMat(in_mat,data_path);
    showMat(out_mat,data_path);
}