#include "orbis_slam/feature_extractor.h"
#include <opencv2/imgproc.hpp>

namespace Orbis {

FeatureExtractor::FeatureExtractor(
    int num_features,
    float scale_factor,
    int num_levels
)
: num_features_(num_features)
, scale_factor_(scale_factor)
, num_levels_(num_levels)
{
    orb_detector_ = cv::ORB::create(
        num_features_,
        scale_factor_,
        num_levels_,
        31,  // edgeThreshold
        0,   // firstLevel
        2,   // WTA_K
        cv::ORB::HARRIS_SCORE,  // scoreType
        31,  // patchSize
        20   // fastThreshold
    );
}

bool FeatureExtractor::extract(
    const cv::Mat& image,
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors
) {
    if (image.empty()) {
        return false;
    }

    // Convert to grayscale if needed
    cv::Mat gray_image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray_image, cv::COLOR_BGRA2GRAY);
    } else {
        gray_image = image;
    }

    // Clear output containers
    keypoints.clear();
    descriptors.release();

    // Detect and compute ORB features
    orb_detector_->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);

    return !keypoints.empty() && !descriptors.empty();
}

} /* namespace Orbis */
