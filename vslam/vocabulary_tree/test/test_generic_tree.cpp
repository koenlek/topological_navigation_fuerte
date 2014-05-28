#include <vocabulary_tree/generic_tree.h>
#include <vocabulary_tree/database.h>
#include <boost/lexical_cast.hpp>
#include <map>
#include <cstdio>
#include <fstream>

int main(int argc, char** argv)
{
  if (argc < 5) {
    printf("Usage: %s vocab.tree vocab.weights signatures.dat objects.dat [NUM_SIGS]\n", argv[0]);
    return 0;
  }

  // Load vocabulary tree
  printf("Loading vocabulary tree\n");
  vt::GenericTree tree(argv[1]);
  size_t dimension = tree.dimension();

  printf("Loading database weights\n");
  vt::Database db(tree.words());
  db.loadWeights(argv[2]);
  
  // Get number of descriptors
  std::ifstream sig_is(argv[3], std::ios::binary);
  int length, num_sigs;
  if (argc == 5) {
    sig_is.seekg(0, std::ios::end);
    length = sig_is.tellg();
    num_sigs = length / (dimension * sizeof(float));
    sig_is.seekg(0, std::ios::beg);
  }
  else {
    num_sigs = boost::lexical_cast<int>(argv[5]);
    length = num_sigs * dimension * sizeof(float);
  }
  printf("Training from %d descriptors\n", num_sigs);

  // Read in descriptors
  printf("Reading descriptors\n");
  cv::Mat features(num_sigs, dimension, cv::DataType<float>::type);
  sig_is.read((char*)features.data, length);

  // Read in object ids
  printf("Reading document ids\n");
  std::ifstream obj_is(argv[4], std::ios::binary);
  std::vector<unsigned int> object_ids(num_sigs);
  obj_is.read((char*)&object_ids[0], num_sigs * sizeof(unsigned int));

  // Compute document vectors
  printf("Computing document vectors\n");
  typedef std::map<unsigned int, vt::Document> DocumentMap;
  DocumentMap documents;
  for (int i = 0; i < num_sigs; ++i) {
    documents[ object_ids[i] ].push_back( tree.quantize(features.row(i)) );
  }
  
  // Add each object (document) to the database
  printf("Inserting documents into database\n");
  for (DocumentMap::const_iterator i = documents.begin(), ie = documents.end(); i != ie; ++i) {
    db.insert(i->second);
  }

  // Now query each document, should get exact matches
  vt::Matches matches;
  for (DocumentMap::const_iterator i = documents.begin(), ie = documents.end(); i != ie; ++i) {
    db.find(i->second, 1, matches);
    printf("%u -> %u, score = %f\n", i->first, matches[0].id, matches[0].score);
  }
  
  return 0;
}
