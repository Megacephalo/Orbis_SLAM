#ifndef _VOCABULARY_MANAGER_H_
#define _VOCABULARY_MANAGER_H_

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <memory>

#include <DBoW3/DBoW3.h>
#include <opencv2/core.hpp>

namespace Orbis {

/**
 * @brief VocabularyManager handles automatic vocabulary creation and management
 *
 * This class provides intelligent vocabulary handling:
 * 1. Loads existing vocabulary if provided
 * 2. Creates vocabulary incrementally during runtime if none exists
 * 3. Saves vocabulary for future use
 */
class VocabularyManager {
public:
    /**
     * @brief Result of vocabulary loading/creation
     */
    enum class Status {
        LOADED_FROM_FILE,      // Successfully loaded existing vocabulary
        CREATED_ONLINE,        // Created new vocabulary (will build incrementally)
        FAILED                 // Failed to load or create
    };

private:
    DBoW3::Vocabulary vocabulary_;
    bool vocabulary_ready_;

    // Online vocabulary building
    std::vector<cv::Mat> training_descriptors_;
    size_t min_features_for_vocab_;
    size_t max_training_features_;
    bool is_building_;

    std::string auto_save_path_;

public:
    /**
     * @brief Construct a new Vocabulary Manager
     *
     * @param vocabulary_path Path to existing vocabulary (can be empty)
     * @param auto_save_path Where to save auto-generated vocabulary
     * @param min_features Minimum features needed before building vocabulary (default: 10000)
     */
    explicit VocabularyManager(
        const std::string& vocabulary_path = "",
        const std::string& auto_save_path = "",
        size_t min_features = 10000
    );

    ~VocabularyManager() = default;

    /**
     * @brief Initialize the vocabulary
     *
     * @return Status indicating how vocabulary was obtained
     */
    Status initialize();

    /**
     * @brief Check if vocabulary is ready for use
     */
    bool isReady() const { return vocabulary_ready_; }

    /**
     * @brief Get reference to the vocabulary
     */
    DBoW3::Vocabulary& getVocabulary() { return vocabulary_; }
    const DBoW3::Vocabulary& getVocabulary() const { return vocabulary_; }

    /**
     * @brief Add features for online vocabulary building
     *
     * @param descriptors ORB descriptors to add to training set
     * @return true if vocabulary was built/updated
     */
    bool addTrainingFeatures(const cv::Mat& descriptors);

    /**
     * @brief Manually trigger vocabulary building from collected features
     *
     * @return true if vocabulary was successfully built
     */
    bool buildVocabulary();

    /**
     * @brief Save current vocabulary to file
     *
     * @param path Path to save vocabulary (uses auto_save_path if empty)
     * @return true if successfully saved
     */
    bool saveVocabulary(const std::string& path = "");

    /**
     * @brief Get status information
     */
    std::string getStatusString() const;

    typedef std::shared_ptr<VocabularyManager> Ptr;

    static VocabularyManager::Ptr create(
        const std::string& vocabulary_path = "",
        const std::string& auto_save_path = "",
        size_t min_features = 10000
    ) {
        return std::make_shared<VocabularyManager>(vocabulary_path, auto_save_path, min_features);
    }

private:
    /**
     * @brief Load vocabulary from file
     */
    bool loadFromFile(const std::string& path);

    /**
     * @brief Create minimal vocabulary for cold start
     */
    bool createMinimalVocabulary();

    /**
     * @brief Get default auto-save path
     */
    std::string getDefaultSavePath() const;
};

} /* namespace Orbis */

#endif /* _VOCABULARY_MANAGER_H_ */
