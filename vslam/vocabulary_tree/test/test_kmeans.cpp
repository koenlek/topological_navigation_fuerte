#include <vocabulary_tree/simple_kmeans.h>
#include <Eigen/Core>
#include <vector>
#include <cstdio>

int main(int argc, char** argv)
{
  static const size_t K = 3;
  
  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 1, K> Feature;
  typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;
  FeatureVector features(1000*K);
  FeatureVector centers;
  std::vector<unsigned int> membership;

  FILE* data = fopen(argv[1], "r");
  for (size_t i = 0; i < features.size(); ++i) {
    Feature& f = features[i];
    fscanf(data, "%lf %lf %lf", &f[0], &f[1], &f[2]);
  }

  vt::SimpleKmeans<Feature> kmeans(Feature::Zero());
  double sse = kmeans.cluster(features, K, centers, membership);

  for (size_t i = 0; i < centers.size(); ++i) {
    printf("%f %f %f\n", centers[i][0], centers[i][1], centers[i][2]);
  }
  printf("sse = %f\n", sse);

  unsigned int pattern[K];
  pattern[0] = membership[0];
  pattern[1] = membership[1];
  pattern[2] = membership[2];
  for (size_t i = 0; i < membership.size(); i+=K) {
    if (membership[i] != pattern[0] ||
        membership[i+1] != pattern[1] ||
        membership[i+2] != pattern[2]) {
      printf("Misassignment!\n");
      return -1;
    }
  }
}
