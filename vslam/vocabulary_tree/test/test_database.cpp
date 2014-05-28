#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <boost/lexical_cast.hpp>
#include <map>
#include <cstdio>
#include <fstream>

static const unsigned int DIMENSION = 176;
typedef Eigen::Matrix<float, 1, DIMENSION> Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

int main(int argc, char** argv)
{
  if (argc < 5) {
    printf("Usage: %s vocab.tree vocab.weights signatures.dat objects.dat [NUM_SIGS]\n", argv[0]);
    return 0;
  }

  // Get number of descriptors
  std::ifstream sig_is(argv[3], std::ios::binary);
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

  // Read in descriptors
  printf("Reading descriptors\n");
  FeatureVector features(num_sigs);
  sig_is.read((char*)&features[0], length);

  // Read in object ids
  printf("Reading document ids\n");
  std::ifstream obj_is(argv[4], std::ios::binary);
  std::vector<unsigned int> object_ids(num_sigs);
  obj_is.read((char*)&object_ids[0], num_sigs * sizeof(unsigned int));

  // Load vocabulary tree
  printf("Loading vocabulary tree\n");
  vt::VocabularyTree<Feature> tree(argv[1]);

  // Compute document vectors
  printf("Computing document vectors\n");
  typedef std::map<unsigned int, vt::Document> DocumentMap;
  DocumentMap documents;
  for (int i = 0; i < num_sigs; ++i) {
    documents[ object_ids[i] ].push_back( tree.quantize(features[i]) );
  }

  printf("Loading database weights\n");
  vt::Database db(tree.words());
  db.loadWeights(argv[2]);
  
  // Add each object (document) to the database
  printf("Inserting documents into database\n");
  for (DocumentMap::const_iterator i = documents.begin(), ie = documents.end(); i != ie; ++i) {
    db.insert(i->second);
  }

  // Now query each document
  vt::Matches matches;
  for (DocumentMap::const_iterator i = documents.begin(), ie = documents.end(); i != ie; ++i) {
    db.find(i->second, 1, matches);
    printf("%u -> %u, score = %f\n", i->first, matches[0].id, matches[0].score);
  }
  
  return 0;
}
