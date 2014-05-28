#ifndef VOCABULARY_TREE_GENERIC_TREE_H
#define VOCABULARY_TREE_GENERIC_TREE_H

#include "vocabulary_tree/vocabulary_tree.h"
#include <opencv/cv.h>

namespace vt {

namespace distance {
/**
 * \brief L2 distance specialization for cv::Mat
 *
 * Note: Currently returns the real distance, not squared! So this won't work well with kmeans.
 */
template<> struct L2<cv::Mat>
{
  typedef double result_type;

  result_type operator()(const cv::Mat& a, const cv::Mat& b) const
  {
    return cv::norm(a, b);
  }
};

} //namespace distance

/**
 * \brief Vocabulary tree wrapper for easy integration with OpenCV features, or when the
 * (dense) descriptor size and/or type isn't known at compile time.
 *
 * \c cv::Mat is used as the feature type. If the feature type is known at compile time,
 * the VocabularyTree template may be considerably more efficient.
 */
class GenericTree : public VocabularyTree<cv::Mat>
{
public:
  /// Constructor, empty tree.
  GenericTree();
  /// Constructor, loads vocabulary from file.
  GenericTree(const std::string& file);

  /// Save vocabulary to a file.
  void save(const std::string& file) const;
  /// Load vocabulary from a file.
  void load(const std::string& file);

  /// Returns the number of elements in a feature used by this tree.
  size_t dimension() const;
};

} //namespace vt

#endif
