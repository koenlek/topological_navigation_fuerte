#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <boost/lexical_cast.hpp>
#include <map>
#include <cstdio>
#include <fstream>

static const unsigned int DIMENSION = 176;

int main(int argc, char** argv)
{
  if (argc < 5) {
    printf("Usage: %s vocab.tree signatures.dat objects.dat output.weights [NUM_SIGS]\n", argv[0]);
    return 0;
  }

  // Load vocabulary tree
  typedef Eigen::Matrix<float, 1, DIMENSION> Feature;
  vt::VocabularyTree<Feature> tree(argv[1]);

  // Get number of descriptors
  std::ifstream sig_is(argv[2], std::ios::binary);
  int length, num_sigs;
  if (argc == 5) {
    sig_is.seekg(0, std::ios::end);
    length = sig_is.tellg();
    num_sigs = length / (DIMENSION * sizeof(float));
    sig_is.seekg(0, std::ios::beg);
  }
  else {
    num_sigs = boost::lexical_cast<int>(argv[5]);
    length = num_sigs * DIMENSION * sizeof(float);
  }
  printf("Training from %d descriptors\n", num_sigs);
  printf("Data length = %d\n", length);

  // Read in descriptors
  typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;
  FeatureVector features(num_sigs);
  sig_is.read((char*)&features[0], length);
  printf("Done reading in descriptors\n");

  // Read in object ids
  std::ifstream obj_is(argv[3], std::ios::binary);
  std::vector<unsigned int> object_ids(num_sigs);
  obj_is.read((char*)&object_ids[0], num_sigs * sizeof(unsigned int));

  // Compute document vectors
  typedef std::map<unsigned int, vt::Document> DocumentMap;
  DocumentMap documents;
  for (int i = 0; i < num_sigs; ++i) {
    documents[ object_ids[i] ].push_back( tree.quantize(features[i]) );
  }
  
  // Add each object (document) to the database
  vt::Database db(tree.words());
  for (DocumentMap::const_iterator i = documents.begin(), ie = documents.end(); i != ie; ++i) {
    db.insert(i->second);
  }

  // Compute and save the word weights
  db.computeTfIdfWeights();
  db.saveWeights(argv[4]);
  
  return 0;
}
