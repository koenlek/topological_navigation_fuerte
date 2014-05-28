#include "vocabulary_tree/vocabulary_tree.h"
#include <cstdio>
#include <boost/lexical_cast.hpp>

int main(int argc, char** argv)
{
  if (argc < 7) {
    printf("Usage: %s A.tree x y z w\n", argv[0]);
    return 0;
  }

  typedef Eigen::Vector4f Feature;
  vt::VocabularyTree<Feature> vtree;
  vtree.load(argv[1]);

  using boost::lexical_cast;
  Feature f(lexical_cast<float>(argv[2]), lexical_cast<float>(argv[3]),
            lexical_cast<float>(argv[4]), lexical_cast<float>(argv[5]));
  vt::Word word = vtree.quantize(f);
  printf("Word = %u\n", word);
  
  return 0;
}
