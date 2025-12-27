#include "orbis_slam/vocabulary_manager.h"
#include <iostream>
#include <chrono>

namespace Orbis {

VocabularyManager::VocabularyManager(
    const std::string& vocabulary_path,
    const std::string& auto_save_path,
    size_t min_features
)
: vocabulary_ready_(false)
, min_features_for_vocab_(min_features)
, max_training_features_(100000)  // Limit to prevent memory issues
, is_building_(false)
, auto_save_path_(auto_save_path.empty() ? getDefaultSavePath() : auto_save_path)
{
    if (!vocabulary_path.empty()) {
        // Try to load from provided path
        if (loadFromFile(vocabulary_path)) {
            vocabulary_ready_ = true;
            std::cout << "[VocabularyManager] Loaded vocabulary from: " << vocabulary_path << std::endl;
        }
    }
}

VocabularyManager::Status VocabularyManager::initialize() {
    if (vocabulary_ready_) {
        return Status::LOADED_FROM_FILE;
    }

    // Check if auto-save path exists
    if (std::filesystem::exists(auto_save_path_)) {
        std::cout << "[VocabularyManager] Found existing auto-saved vocabulary: "
                  << auto_save_path_ << std::endl;
        if (loadFromFile(auto_save_path_)) {
            vocabulary_ready_ = true;
            return Status::LOADED_FROM_FILE;
        }
    }

    // Create minimal vocabulary for cold start
    std::cout << "[VocabularyManager] No vocabulary found. Creating minimal vocabulary..." << std::endl;
    std::cout << "[VocabularyManager] Full vocabulary will be built online as features are collected." << std::endl;
    std::cout << "[VocabularyManager] Collecting features (need " << min_features_for_vocab_
              << " before building)..." << std::endl;

    if (createMinimalVocabulary()) {
        vocabulary_ready_ = true;
        return Status::CREATED_ONLINE;
    }

    return Status::FAILED;
}

bool VocabularyManager::addTrainingFeatures(const cv::Mat& descriptors) {
    if (descriptors.empty() || is_building_) {
        return false;
    }

    // Add descriptors to training set
    for (int i = 0; i < descriptors.rows; ++i) {
        training_descriptors_.push_back(descriptors.row(i).clone());

        // Limit total features to prevent memory issues
        if (training_descriptors_.size() >= max_training_features_) {
            break;
        }
    }

    // Check if we have enough features to build vocabulary
    if (training_descriptors_.size() >= min_features_for_vocab_ && !vocabulary_ready_) {
        std::cout << "[VocabularyManager] Collected " << training_descriptors_.size()
                  << " features. Building vocabulary..." << std::endl;

        if (buildVocabulary()) {
            std::cout << "[VocabularyManager] Vocabulary built successfully!" << std::endl;
            vocabulary_ready_ = true;

            // Save for future use
            if (saveVocabulary()) {
                std::cout << "[VocabularyManager] Vocabulary saved to: " << auto_save_path_ << std::endl;
            }

            // Clear training data to free memory
            training_descriptors_.clear();
            training_descriptors_.shrink_to_fit();

            return true;
        }
    }

    return false;
}

bool VocabularyManager::buildVocabulary() {
    if (training_descriptors_.empty()) {
        std::cerr << "[VocabularyManager] No training features available!" << std::endl;
        return false;
    }

    is_building_ = true;

    try {
        auto start = std::chrono::high_resolution_clock::now();

        // Convert to format DBoW3 expects: vector<vector<cv::Mat>>
        // DBoW3 expects each inner vector to represent descriptors from one image
        // We'll group our descriptors into batches
        std::vector<std::vector<cv::Mat>> training_data;
        std::vector<cv::Mat> current_batch;

        const size_t batch_size = 100;  // Group descriptors in batches of 100
        for (const auto& desc : training_descriptors_) {
            current_batch.push_back(desc);
            if (current_batch.size() >= batch_size) {
                training_data.push_back(current_batch);
                current_batch.clear();
            }
        }
        // Add remaining descriptors
        if (!current_batch.empty()) {
            training_data.push_back(current_batch);
        }

        // Build vocabulary with reasonable parameters
        // k = 10 (branching factor), L = 5 (depth), weighting and scoring
        int k = 10;        // Branching factor
        int L = 5;         // Depth levels
        DBoW3::WeightingType weighting = DBoW3::TF_IDF;
        DBoW3::ScoringType scoring = DBoW3::L1_NORM;

        std::cout << "[VocabularyManager] Creating vocabulary tree (k=" << k << ", L=" << L << ")..." << std::endl;

        vocabulary_.create(training_data, k, L, weighting, scoring);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);

        std::cout << "[VocabularyManager] Vocabulary created in " << duration.count()
                  << " seconds with " << vocabulary_.size() << " visual words" << std::endl;

        is_building_ = false;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[VocabularyManager] Failed to build vocabulary: " << e.what() << std::endl;
        is_building_ = false;
        return false;
    }
}

bool VocabularyManager::saveVocabulary(const std::string& path) {
    std::string save_path = path.empty() ? auto_save_path_ : path;

    try {
        // Create directory if it doesn't exist
        std::filesystem::path file_path(save_path);
        std::filesystem::create_directories(file_path.parent_path());

        vocabulary_.save(save_path);
        std::cout << "[VocabularyManager] Vocabulary saved to: " << save_path << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[VocabularyManager] Failed to save vocabulary: " << e.what() << std::endl;
        return false;
    }
}

bool VocabularyManager::loadFromFile(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        std::cerr << "[VocabularyManager] Vocabulary file not found: " << path << std::endl;
        return false;
    }

    try {
        vocabulary_.load(path);

        if (vocabulary_.size() == 0) {
            std::cerr << "[VocabularyManager] Loaded vocabulary is empty!" << std::endl;
            return false;
        }

        std::cout << "[VocabularyManager] Loaded vocabulary with " << vocabulary_.size()
                  << " visual words" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[VocabularyManager] Failed to load vocabulary: " << e.what() << std::endl;
        return false;
    }
}

bool VocabularyManager::createMinimalVocabulary() {
    // Create a very minimal vocabulary as placeholder
    // This allows the system to start, and will be replaced once enough features are collected

    try {
        // Create minimal vocabulary with just a few dummy features
        // DBoW3 expects vector<vector<cv::Mat>> format
        std::vector<std::vector<cv::Mat>> minimal_training_data;

        // Create some random ORB-like descriptors (32 bytes each)
        // Group them into 10 "images" with 10 descriptors each
        for (int img = 0; img < 10; ++img) {
            std::vector<cv::Mat> image_descriptors;
            for (int i = 0; i < 10; ++i) {
                cv::Mat desc(1, 32, CV_8U);
                cv::randu(desc, cv::Scalar(0), cv::Scalar(255));
                image_descriptors.push_back(desc);
            }
            minimal_training_data.push_back(image_descriptors);
        }

        // Create very small vocabulary (k=5, L=2)
        vocabulary_.create(minimal_training_data, 5, 2, DBoW3::TF_IDF, DBoW3::L1_NORM);

        std::cout << "[VocabularyManager] Created minimal placeholder vocabulary" << std::endl;
        std::cout << "[VocabularyManager] Note: Loop closure will be limited until full vocabulary is built" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[VocabularyManager] Failed to create minimal vocabulary: " << e.what() << std::endl;
        return false;
    }
}

std::string VocabularyManager::getDefaultSavePath() const {
    // Save to user's home directory
    const char* home = std::getenv("HOME");
    if (!home) {
        home = "/tmp";  // Fallback
    }

    std::filesystem::path save_dir = std::filesystem::path(home) / ".orbis_slam";
    return (save_dir / "auto_vocabulary.dbow3").string();
}

std::string VocabularyManager::getStatusString() const {
    std::stringstream ss;

    if (vocabulary_ready_) {
        ss << "Vocabulary ready (" << vocabulary_.size() << " visual words)";
    } else if (!training_descriptors_.empty()) {
        ss << "Collecting features (" << training_descriptors_.size()
           << " / " << min_features_for_vocab_ << ")";
    } else {
        ss << "Vocabulary not initialized";
    }

    return ss.str();
}

} /* namespace Orbis */
