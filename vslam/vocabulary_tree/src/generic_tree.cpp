#include <vocabulary_tree/generic_tree.h>
#include <stdexcept>
#include <boost/format.hpp>

namespace vt {

GenericTree::GenericTree()
{
}

GenericTree::GenericTree(const std::string& file)
{
  load(file);
}

/// @todo Currently assuming float. Really need more info in the save format.
// Kinda sucks that we need to wrap these functions.
void GenericTree::save(const std::string& file) const
{
  assert( initialized() );

  std::ofstream out(file.c_str(), std::ios_base::binary);
  out.write((char*)(&k_), sizeof(uint32_t));
  out.write((char*)(&levels_), sizeof(uint32_t));
  uint32_t size = centers_.size();
  out.write((char*)(&size), sizeof(uint32_t));
  // This is pretty hacky! Retrieve the start and end of the block of data.
  const uchar* start = centers_[0].datastart;
  const uchar* end = centers_[0].dataend;
  out.write((const char*)start, end - start);
  out.write((const char*)(&valid_centers_[0]), valid_centers_.size());
}

void GenericTree::load(const std::string& file)
{
  clear();

  std::ifstream in;
  in.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);

  uint32_t size;
  try {
    in.open(file.c_str(), std::ios_base::binary);
    in.read((char*)(&k_), sizeof(uint32_t));
    in.read((char*)(&levels_), sizeof(uint32_t));
    in.read((char*)(&size), sizeof(uint32_t));

    // Use arithmetic on file size to get the descriptor length, ugh. Assuming float.
    in.seekg(0, std::ios::end);
    int length = in.tellg();
    int dimension = ((length - 12)/size - sizeof(uint8_t)) / sizeof(float);
    in.seekg(12, std::ios::beg);

    // Read in centers as one big cv::Mat to preserve data locality.
    cv::Mat all(size, dimension, cv::DataType<float>::type);
    assert(all.isContinuous());
    in.read((char*)all.data, size * dimension * sizeof(float));
    // Now add cv::Mat centers that point into the big block of data.
    centers_.reserve(size);
    for (int i = 0; i < all.rows; ++i)
      centers_.push_back(all.row(i));

    // Read in valid centers as usual
    valid_centers_.resize(size);
    in.read((char*)(&valid_centers_[0]), valid_centers_.size());
    assert(in.tellg() == length);
  }
  catch (std::ifstream::failure& e) {
    throw std::runtime_error( (boost::format("Failed to load vocabulary tree file '%s'") % file).str() );
  }

  setNodeCounts();
  assert(size == num_words_ + word_start_);
}

size_t GenericTree::dimension() const
{
  assert( initialized() );
  return centers_[0].cols;
}

} //namespace vt
