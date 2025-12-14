#ifndef _FEATURE_EXTRACTOR_H_
#define _FEATURE_EXTRACTOR_H_

#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace Orbis {

/**
 * @brief FeatureExtractor class for detecting and computing ORB features
 *
 * This class wraps OpenCV's ORB detector to extract features from images
 * for loop closure detection. It provides consistent feature extraction
 * across the SLAM pipeline.
 */
class FeatureExtractor {
private:
    cv::Ptr<cv::ORB> orb_detector_;
    int num_features_;
    float scale_factor_;
    int num_levels_;

public:
    /**
     * @brief Construct a new Feature Extractor
     *
     * @param num_features Number of features to extract (default: 1000)
     * @param scale_factor Pyramid decimation ratio (default: 1.2)
     * @param num_levels Number of pyramid levels (default: 8)
     */
    explicit FeatureExtractor(
        int num_features = 1000,
        float scale_factor = 1.2f,
        int num_levels = 8
    );

    ~FeatureExtractor() = default;

    /**
     * @brief Extract ORB features from an image
     *
     * @param image Input image (grayscale or BGR)
     * @param keypoints Output keypoints
     * @param descriptors Output descriptors (CV_8U)
     * @return true if features were successfully extracted
     */
    bool extract(
        const cv::Mat& image,
        std::vector<cv::KeyPoint>& keypoints,
        cv::Mat& descriptors
    );

    /**
     * @brief Get the number of configured features
     */
    int getNumFeatures() const { return num_features_; }

    /**
     * @brief Get the scale factor
     */
    float getScaleFactor() const { return scale_factor_; }

    /**
     * @brief Get the number of pyramid levels
     */
    int getNumLevels() const { return num_levels_; }

    typedef std::shared_ptr<FeatureExtractor> Ptr;

    static FeatureExtractor::Ptr create(
        int num_features = 1000,
        float scale_factor = 1.2f,
        int num_levels = 8
    ) {
        return std::make_shared<FeatureExtractor>(num_features, scale_factor, num_levels);
    }
};

} /* namespace Orbis */

#endif /* _FEATURE_EXTRACTOR_H_ */
