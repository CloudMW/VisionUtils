//
// Created by mfy on 2025/12/11.
//

#ifndef VISIONUTILS_CV_UTILS_ARUCO_HPP
#define VISIONUTILS_CV_UTILS_ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace cv_utils::aruco {
    /*!
     * @brief 识别aruco码
     * @param img 输入图像
     * @param markerIds id序号
     * @param markerCorners 不同aruco码的角点坐标
     * @param out_mat 输出图像，带有aruco码信息
     * @code
     * cv::Mat in_mat = cv::imread(data_path);
     * std::vector<int> markerIds;
     * std::vector<std::vector<cv::Point2f>> markerCorners;
     * cv::Mat out_mat;
     * cv_utils::aruco::detectMarkes(in_mat,markerIds,markerCorners,out_mat);
     * @endcode
     */
    void detectMarkes(const cv::Mat &img, std::vector<int> markerIds,
                      std::vector<std::vector<cv::Point2f> > &markerCorners, cv::Mat &out_mat) {
        std::vector<std::vector<cv::Point2f> > rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(img, markerCorners, markerIds, rejectedCandidates);
        out_mat = img.clone();
        cv::aruco::drawDetectedMarkers(out_mat, markerCorners, markerIds);
    }
}
#endif //VISIONUTILS_CV_UTILS_ARUCO_HPP
