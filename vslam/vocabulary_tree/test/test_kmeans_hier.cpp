#include <vocabulary_tree/simple_kmeans.h>
#include <Eigen/Core>
#include <vector>
#include <cstdio>
#include <fstream>

int main(int argc, char** argv)
{
  static const size_t K = 5;
  typedef Eigen::Vector2f Feature;
  typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;
  
  FeatureVector features;
  FeatureVector centers;
  std::vector<unsigned int> membership;

  std::ifstream infile(argv[1]);
  while (infile.good()) {
    Feature f;
    infile >> f[0] >> f[1];
    if (infile.good())
      features.push_back(f);
  }
  
  vt::SimpleKmeans<Feature> kmeans(Feature::Zero());
#if 1
  kmeans.setRestarts(3);
#else
  kmeans.setInitMethod(vt::InitGiven());
  centers.reserve(5);
  centers.push_back(Feature(1000, 0));
  centers.push_back(Feature(306, 963));
  centers.push_back(Feature(-812, 594));
  centers.push_back(Feature(-812, -594));
  centers.push_back(Feature(306, -963));
#endif

  double sse = kmeans.cluster(features, K, centers, membership);

  for (size_t i = 0; i < centers.size(); ++i) {
    printf("%f %f\n", centers[i][0], centers[i][1]);
  }
  printf("sse = %f\n", sse);
}
