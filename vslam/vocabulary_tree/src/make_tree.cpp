#include <vocabulary_tree/tree_builder.h>
#include <boost/lexical_cast.hpp>
#include <cstdio>
#include <fstream>

static const unsigned int DIMENSION = 176;
static const uint32_t K = 10;
static const uint32_t LEVELS = 5;



int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s signatures.dat output.tree [NUM_SIGS]\n", argv[0]);
    return 0;
  }

  std::ifstream sig_is(argv[1], std::ios::binary);
  std::string tree_file = argv[2];

  // Get number of descriptors
  int length, num_sigs;
  if (argc == 3) {
    sig_is.seekg(0, std::ios::end);
    length = sig_is.tellg();
    num_sigs = length / (DIMENSION * sizeof(float));
    sig_is.seekg(0, std::ios::beg);
  }
  else {
    num_sigs = boost::lexical_cast<int>(argv[3]);
    length = num_sigs * DIMENSION * sizeof(float);
  }
  printf("Training from %d descriptors\n", num_sigs);
  printf("Data length = %d\n", length);

  // Read in descriptors
  typedef Eigen::Matrix<float, 1, DIMENSION> Feature;
  typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;
  FeatureVector features(num_sigs);
  sig_is.read((char*)&features[0], length);
  printf("Done reading in descriptors\n");

  // Create tree
  vt::TreeBuilder<Feature> builder(Feature::Zero());
  builder.kmeans().setRestarts(5);
  builder.build(features, K, LEVELS);
  printf("%u centers\n", builder.tree().centers().size());
  builder.tree().save(tree_file);
  
  return 0;
}
