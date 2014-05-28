#ifndef VOCABULARY_TREE_TREE_BUILDER_H
#define VOCABULARY_TREE_TREE_BUILDER_H

#include "vocabulary_tree/mutable_tree.h"
#include "vocabulary_tree/simple_kmeans.h"
#include <deque>
//#include <cstdio> //DEBUG

namespace vt {

/**
 * \brief Class for building a new vocabulary by hierarchically clustering
 * a set of training features.
 */
template<class Feature, class Distance = distance::L2<Feature>,
         class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class TreeBuilder
{
public:
  typedef MutableVocabularyTree<Feature, Distance, FeatureAllocator> Tree;
  typedef SimpleKmeans<Feature, Distance, FeatureAllocator> Kmeans;
  typedef std::vector<Feature, FeatureAllocator> FeatureVector;

  /**
   * \brief Constructor
   *
   * \param zero Object representing zero in the feature space
   * \param d    Functor for calculating squared distance
   */
  TreeBuilder(const Feature& zero = Feature(), Distance d = Distance());

  /**
   * \brief Build a new vocabulary tree.
   *
   * The number of words in the resulting vocabulary is at most k ^ levels.
   *
   * \param training_features The set of training features to cluster.
   * \param k                 The branching factor, or max children of any node.
   * \param levels            The number of levels in the tree.
   */
  void build(const FeatureVector& training_features, uint32_t k, uint32_t levels);

  /// Get the built vocabulary tree.
  const Tree& tree() const { return tree_; }

  /// Get the k-means clusterer.
  Kmeans& kmeans() { return kmeans_; }
  /// Get the k-means clusterer.
  const Kmeans& kmeans() const { return kmeans_; }

protected:
  Tree tree_;
  Kmeans kmeans_;
  Feature zero_;
};


template<class Feature, class Distance, class FeatureAllocator>
TreeBuilder<Feature, Distance, FeatureAllocator>::TreeBuilder(const Feature& zero, Distance d)
  : tree_(d),
    kmeans_(zero, d),
    zero_(zero)
{
}

template<class Feature, class Distance, class FeatureAllocator>
void TreeBuilder<Feature, Distance, FeatureAllocator>::build(const FeatureVector& training_features,
                                                             uint32_t k, uint32_t levels)
{
  // Initial setup and memory allocation for the tree
  tree_.clear();
  tree_.setSize(levels, k);
  tree_.centers().reserve(tree_.nodes());
  tree_.validCenters().reserve(tree_.nodes());

  // We keep a queue of disjoint feature subsets to cluster.
  // Feature* is used to avoid copying features.
  std::deque< std::vector<Feature*> > subset_queue(1);

  // At first the queue contains one "subset" containing all the features.
  std::vector<Feature*> &feature_ptrs = subset_queue.front();
  feature_ptrs.reserve(training_features.size());
  BOOST_FOREACH(const Feature& f, training_features)
    feature_ptrs.push_back(const_cast<Feature*>(&f));

  FeatureVector centers; // always size k
  for (uint32_t level = 0; level < levels; ++level) {
    //printf("# Level %u\n", level);
    std::vector<unsigned int> membership;

    for (size_t i = 0, ie = subset_queue.size(); i < ie; ++i) {
      std::vector<Feature*> &subset = subset_queue.front();
      //printf("#\tClustering subset of size %u\n", subset.size());

      // If the subset already has k or fewer elements, just use those as the centers.
      if (subset.size() <= k) {
        for (size_t j = 0; j < subset.size(); ++j) {
          tree_.centers().push_back(*subset[j]);
          tree_.validCenters().push_back(1);
        }
        // Mark non-existent centers as invalid.
        tree_.centers().insert(tree_.centers().end(), k - subset.size(), zero_);
        tree_.validCenters().insert(tree_.validCenters().end(), k - subset.size(), 0);

        // Push k empty subsets into the queue so all children get marked invalid.
        subset_queue.pop_front();
        subset_queue.insert(subset_queue.end(), k, std::vector<Feature*>());
      }
      else {
        // Cluster the current subset into k centers.
        kmeans_.clusterPointers(subset, k, centers, membership);

        // Add the centers and mark them as valid.
        tree_.centers().insert(tree_.centers().end(), centers.begin(), centers.end());
        tree_.validCenters().insert(tree_.validCenters().end(), k, 1);

        // Partition the current subset into k new subsets based on the cluster assignments.
        std::vector< std::vector<Feature*> > new_subsets(k);
        for (size_t j = 0; j < subset.size(); ++j) {
          new_subsets[ membership[j] ].push_back(subset[j]);
        }

        // Update the queue
        subset_queue.pop_front();
        subset_queue.insert(subset_queue.end(), new_subsets.begin(), new_subsets.end());
      }
    }
    //printf("# centers so far = %u\n", tree_.centers().size());
  }
}

} //namespace vt

#endif
