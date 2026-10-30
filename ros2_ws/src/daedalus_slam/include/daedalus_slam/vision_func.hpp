#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


void feature_extraction(const cv::Mat &img_1, const cv::Mat &img_2,
                        std::vector<cv::KeyPoint> &keypoints_1,
                        std::vector<cv::KeyPoint> &keypoints_2,
                        std::vector<cv::DMatch> &good_matches);
