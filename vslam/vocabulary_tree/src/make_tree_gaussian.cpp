#include <vocabulary_tree/tree_builder.h>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <cstdio>
#include <fstream>

static const uint32_t K = 5;
static const uint32_t LEVELS = 3;

typedef Eigen::Vector2f Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s samples.txt output.tree\n", argv[0]);
    return 0;
  }

  // Parse command-line args
  std::ifstream infile(argv[1]);
  std::string tree_file = argv[2];

  // Read in descriptors
  FeatureVector features;
  while (infile.good()) {
    Feature f;
    infile >> f[0] >> f[1];
    if (infile.good())
      features.push_back(f);
  }
  printf("# Training from %d descriptors\n", features.size());

  // Create tree
  vt::TreeBuilder<Feature> builder(Feature::Zero());
  builder.kmeans().setRestarts(5);
  builder.build(features, K, LEVELS);
  builder.tree().save(tree_file);
  printf("# %u centers\n", builder.tree().centers().size());

  // Print out centers
  BOOST_FOREACH(const Feature& f, builder.tree().centers()) {
    printf("%f %f\n", f[0], f[1]);
  }
  
  return 0;
}
