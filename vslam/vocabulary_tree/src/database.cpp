#include "vocabulary_tree/database.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <boost/format.hpp>

namespace vt {

Database::Database(uint32_t num_words)
  : word_files_(num_words),
    word_weights_(num_words, 1.0f)
{
}

DocId Database::insert(const std::vector<Word>& document)
{
  /// @todo Evaluate whether sorting words makes much difference in speed
  DocId doc_id = database_vectors_.size();

  // For each word, retrieve its inverted file and increment the count for doc_id.
  for (std::vector<Word>::const_iterator it = document.begin(), end = document.end(); it != end; ++it) {
    Word word = *it;
    InvertedFile& file = word_files_[word];
    if (file.empty() || file.back().id != doc_id)
      file.push_back(WordFrequency(doc_id, 1));
    else
      file.back().count++;
  }

  // Precompute the document vector to compare queries against.
  database_vectors_.resize(doc_id + 1);
  computeVector(document, database_vectors_.back());
  
  return doc_id;
}

void Database::find(const std::vector<Word>& document, size_t N, std::vector<Match>& matches) const
{
  DocumentVector query;
  computeVector(document, query);

  // Accumulate the best N matches
  using namespace boost::accumulators;
  typedef tag::tail<left> bestN_tag;
  accumulator_set<Match, features<bestN_tag> > acc(bestN_tag::cache_size = N);

  /// @todo Try only computing distances against documents sharing at least one word
  for (DocId i = 0; i < (DocId)database_vectors_.size(); ++i) {
    float distance = sparseDistance(query, database_vectors_[i]);
    acc( Match(i, distance) );
  }

  extractor<bestN_tag> bestN;
  matches.resize( std::min(N, database_vectors_.size()) );
  std::copy(bestN(acc).begin(), bestN(acc).end(), matches.begin());
}

DocId Database::findAndInsert(const std::vector<Word>& document, size_t N, std::vector<Match>& matches)
{
  /// @todo Can this be accelerated? Could iterate over words only once?
  find(document, N, matches);
  return insert(document);
}

void Database::computeTfIdfWeights(float default_weight)
{
  float N = (float)database_vectors_.size();
  size_t num_words = word_files_.size();
  for (size_t i = 0; i < num_words; ++i) {
    size_t Ni = word_files_[i].size();
    if (Ni != 0)
      word_weights_[i] = std::log(N / Ni);
    else
      word_weights_[i] = default_weight;
  }
}

void Database::saveWeights(const std::string& file) const
{
  std::ofstream out(file.c_str(), std::ios_base::binary);
  uint32_t num_words = word_weights_.size();
  out.write((char*)(&num_words), sizeof(uint32_t));
  out.write((char*)(&word_weights_[0]), num_words * sizeof(float));
}

void Database::loadWeights(const std::string& file)
{
  std::ifstream in;
  in.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);

  try {
    in.open(file.c_str(), std::ios_base::binary);
    uint32_t num_words = 0;
    in.read((char*)(&num_words), sizeof(uint32_t));
    word_files_.resize(num_words); // Inverted files start out empty
    word_weights_.resize(num_words);
    in.read((char*)(&word_weights_[0]), num_words * sizeof(float));
  }
  catch (std::ifstream::failure& e) {
    throw std::runtime_error( (boost::format("Failed to load vocabulary weights file '%s'") % file).str() );
  }
}

void Database::computeVector(const std::vector<Word>& document, DocumentVector& v) const
{
  for (std::vector<Word>::const_iterator it = document.begin(), end = document.end(); it != end; ++it) {
    Word word = *it;
    v[word] += word_weights_[word];
  }
  normalize(v);
}

void Database::normalize(DocumentVector& v)
{
  float sum = 0.0f;
  for (DocumentVector::iterator i = v.begin(), ie = v.end(); i != ie; ++i)
    sum += i->second;
  float inv_sum = 1.0f / sum;
  for (DocumentVector::iterator i = v.begin(), ie = v.end(); i != ie; ++i)
    i->second *= inv_sum;
}

float Database::sparseDistance(const DocumentVector& v1, const DocumentVector& v2)
{
  float distance = 0.0f;
  DocumentVector::const_iterator i1 = v1.begin(), i1e = v1.end();
  DocumentVector::const_iterator i2 = v2.begin(), i2e = v2.end();

  while (i1 != i1e && i2 != i2e) {
    if (i2->first < i1->first) {
      distance += i2->second;
      ++i2;
    }
    else if (i1->first < i2->first) {
      distance += i1->second;
      ++i1;
    }
    else {
      distance += fabs(i1->second - i2->second);
      ++i1; ++i2;
    }
  }

  while (i1 != i1e) {
    distance += i1->second;
    ++i1;
  }

  while (i2 != i2e) {
    distance += i2->second;
    ++i2;
  }
  
  return distance;
}

} //namespace vt
