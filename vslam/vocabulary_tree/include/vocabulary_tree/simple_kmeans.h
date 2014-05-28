#ifndef VOCABULARY_TREE_SIMPLE_KMEANS_H
#define VOCABULARY_TREE_SIMPLE_KMEANS_H

#include "vocabulary_tree/distance.h"
#include "vocabulary_tree/feature_allocator.h"
#include <boost/function.hpp>
#include <boost/foreach.hpp>
#include <algorithm>
#include <vector>
#include <limits>

namespace vt {

// Forward declare function objects for choosing the initial centers
struct InitRandom;
struct InitGiven;
/// @todo InitKmeanspp

/**
 * \brief Class for performing K-means clustering, optimized for a particular feature type and metric.
 *
 * The standard Lloyd's algorithm is used. By default, cluster centers are initialized randomly.
 */
template<class Feature, class Distance = distance::L2<Feature>,
         class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class SimpleKmeans
{
public:
  typedef typename Distance::result_type squared_distance_type;
  typedef boost::function<void(const std::vector<Feature*>&, size_t, std::vector<Feature, FeatureAllocator>&, Distance)> Initializer;

  /**
   * \brief Constructor
   *
   * \param zero Object representing zero in the feature space
   * \param d    Functor for calculating squared distance
   * 
   * @todo FeatureAllocator parameter
   */
  SimpleKmeans(const Feature& zero = Feature(), Distance d = Distance());

  /// Set function object used to choose initial cluster centers.
  void setInitMethod(const Initializer& init) { choose_centers_ = init; }

  size_t getMaxIterations() const { return max_iterations_; }
  void setMaxIterations(size_t iters) { max_iterations_ = iters; }

  size_t getRestarts() const { return restarts_; }
  void setRestarts(size_t restarts) { restarts_ = restarts; }

  /**
   * \brief Partition a set of features into k clusters.
   *
   * \param      features   The features to be clustered.
   * \param      k          The number of clusters.
   * \param[out] centers    A set of k cluster centers.
   * \param[out] membership Cluster assignment for each feature
   */
  squared_distance_type cluster(const std::vector<Feature, FeatureAllocator>& features, size_t k,
                                std::vector<Feature, FeatureAllocator>& centers,
                                std::vector<unsigned int>& membership) const;

  /**
   * \brief Partition a set of features into k clusters.
   *
   * This version is more convenient for hierarchical clustering, as you do not have to copy
   * feature objects.
   *
   * \param      features   The features to be clustered.
   * \param      k          The number of clusters.
   * \param[out] centers    A set of k cluster centers.
   * \param[out] membership Cluster assignment for each feature
   */
  squared_distance_type clusterPointers(const std::vector<Feature*>& features, size_t k,
                                        std::vector<Feature, FeatureAllocator>& centers,
                                        std::vector<unsigned int>& membership) const;
  
private:
  
  squared_distance_type clusterOnce(const std::vector<Feature*>& features, size_t k,
                                    std::vector<Feature, FeatureAllocator>& centers,
                                    std::vector<unsigned int>& membership) const;
  
  Feature zero_;
  Distance distance_;
  Initializer choose_centers_;
  size_t max_iterations_;
  size_t restarts_;
};


template < class Feature, class Distance, class FeatureAllocator >
SimpleKmeans<Feature, Distance, FeatureAllocator>::SimpleKmeans(const Feature& zero, Distance d)
  : zero_(zero),
    distance_(d),
    choose_centers_(InitRandom()),
    max_iterations_(100),
    restarts_(1)
{
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::cluster(const std::vector<Feature, FeatureAllocator>& features, size_t k,
                                                           std::vector<Feature, FeatureAllocator>& centers,
                                                           std::vector<unsigned int>& membership) const
{
  std::vector<Feature*> feature_ptrs;
  feature_ptrs.reserve(features.size());
  BOOST_FOREACH(const Feature& f, features)
    feature_ptrs.push_back(const_cast<Feature*>(&f));
  return clusterPointers(feature_ptrs, k, centers, membership);
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::clusterPointers(const std::vector<Feature*>& features, size_t k,
                                                                   std::vector<Feature, FeatureAllocator>& centers,
                                                                   std::vector<unsigned int>& membership) const
{
  std::vector<Feature, FeatureAllocator> new_centers(centers);
  new_centers.resize(k);
  std::vector<unsigned int> new_membership(features.size());

  squared_distance_type least_sse = std::numeric_limits<squared_distance_type>::max();
  for (size_t starts = 0; starts < restarts_; ++starts) {
    choose_centers_(features, k, new_centers, distance_);
    
    squared_distance_type sse = clusterOnce(features, k, new_centers, new_membership);
    if (sse < least_sse) {
      least_sse = sse;
      centers = new_centers;
      membership = new_membership;
    }
  }

  return least_sse;
}

template < class Feature, class Distance, class FeatureAllocator >
typename SimpleKmeans<Feature, Distance, FeatureAllocator>::squared_distance_type
SimpleKmeans<Feature, Distance, FeatureAllocator>::clusterOnce(const std::vector<Feature*>& features, size_t k,
                                                               std::vector<Feature, FeatureAllocator>& centers,
                                                               std::vector<unsigned int>& membership) const
{
  std::vector<size_t> new_center_counts(k);
  std::vector<Feature, FeatureAllocator> new_centers(k);
  
  for (size_t iter = 0; iter < max_iterations_; ++iter) {
    // Zero out new centers and counts
    std::fill(new_center_counts.begin(), new_center_counts.end(), 0);
    std::fill(new_centers.begin(), new_centers.end(), zero_);
    bool is_stable = true;

    // Assign data objects to current centers
    for (size_t i = 0; i < features.size(); ++i) {
      squared_distance_type d_min = std::numeric_limits<squared_distance_type>::max();
      unsigned int nearest = -1;
      // Find the nearest cluster center to feature i
      for (unsigned int j = 0; j < k; ++j) {
        squared_distance_type distance = distance_(*features[i], centers[j]);
        if (distance < d_min) {
          d_min = distance;
          nearest = j;
        }
      }

      // Assign feature i to the cluster it is nearest to
      if (membership[i] != nearest) {
        is_stable = false;
        membership[i] = nearest;
      }

      // Accumulate the cluster center and its membership count
      new_centers[nearest] += *features[i];
      ++new_center_counts[nearest];
    }
    if (is_stable) break;
    
    // Assign new centers
    for (size_t i = 0; i < k; ++i) {
      if (new_center_counts[i] > 0) {
        centers[i] = new_centers[i] / new_center_counts[i];
      }
      else {
        // Choose a new center randomly from the input features
        unsigned int index = rand() % features.size();
        centers[i] = *features[index];
      }
    }
  }

  // Return the sum squared error
  /// @todo Kahan summation?
  squared_distance_type sse = squared_distance_type();
  for (size_t i = 0; i < features.size(); ++i) {
    sse += distance_(*features[i], centers[membership[i]]);
  }
  return sse;
}


/**
 * \brief Initializer for K-means that randomly selects k features as the cluster centers.
 */
struct InitRandom
{
  template<class Feature, class Distance, class FeatureAllocator>
  void operator()(const std::vector<Feature*>& features, size_t k, std::vector<Feature, FeatureAllocator>& centers, Distance distance)
  {
    // Construct a random permutation of the features using a Fisher-Yates shuffle
    std::vector<Feature*> features_perm = features;
    for (size_t i = features.size(); i > 1; --i) {
      size_t k = rand() % i;
      std::swap(features_perm[i-1], features_perm[k]);
    }
    // Take the first k permuted features as the initial centers
    for (size_t i = 0; i < centers.size(); ++i)
      centers[i] = *features_perm[i];
  }
};

/**
 * \brief Dummy initializer for K-means that leaves the centers as-is.
 */
struct InitGiven
{
  template<class Feature, class Distance, class FeatureAllocator>
  void operator()(const std::vector<Feature*>& features, size_t k, std::vector<Feature, FeatureAllocator>& centers, Distance distance)
  {
    // Do nothing!
  }
};

} //namespace vt

#endif
