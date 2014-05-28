#include "vocabulary_tree/mutable_tree.h"

int main(int argc, char** argv)
{
  typedef Eigen::Vector4f Feature;

  vt::MutableVocabularyTree<Feature> vtree;
  vtree.setSize(2, 2);
  // Level 1
  vtree.centers().push_back(Feature(-5.0f, -5.0f, -5.0f, -5.0f));
  vtree.centers().push_back(Feature(5.0f, 5.0f, 5.0f, 5.0f));
  // Level 2
  vtree.centers().push_back(Feature(-4.0f, -4.0f, -4.0f, -4.0f));
  vtree.centers().push_back(Feature(-6.0f, -6.0f, -6.0f, -6.0f));
  vtree.centers().push_back(Feature(4.0f, 4.0f, 4.0f, 4.0f));
  vtree.centers().push_back(Feature(6.0f, 6.0f, 6.0f, 6.0f));

  // All nodes are valid
  vtree.validCenters().resize(6, 1);
  
  vtree.save("test.tree");

  return 0;
}
