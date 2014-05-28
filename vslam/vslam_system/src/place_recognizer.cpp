#include <vslam_system/place_recognizer.h>

namespace vslam {

PlaceRecognizer::PlaceRecognizer(const std::string& tree_file,
                                 const std::string& weights_file)
  : tree_(tree_file)
{
  database_.loadWeights(weights_file);
}

void PlaceRecognizer::insert(const frame_common::Frame& frame, uint32_t id)
{
  vt::Document words(frame.dtors.rows);
  for (int i = 0; i < frame.dtors.rows; ++i) {
    words[i] = tree_.quantize(frame.dtors.row(i));
  }

  vt::DocId doc_id = database_.insert(words);
  assert(doc_id == user_ids_.size());
  user_ids_.push_back(id);
}

void PlaceRecognizer::findAndInsert(const frame_common::Frame& frame, uint32_t id,
                                    const FrameVector& all_frames, size_t N,
                                    std::vector<const frame_common::Frame*>& matches)
{
  vt::Document words(frame.dtors.rows);
  for (int i = 0; i < frame.dtors.rows; ++i) {
    words[i] = tree_.quantize(frame.dtors.row(i));
  }

  vt::Matches doc_matches;
  vt::DocId doc_id = database_.findAndInsert(words, N, doc_matches);
  assert(doc_id == user_ids_.size());
  user_ids_.push_back(id);
  
  matches.resize(doc_matches.size());
  for (size_t i = 0; i < matches.size(); ++i) {
    matches[i] = &all_frames[ user_ids_[doc_matches[i].id] ];
  }
}

} //namespace vslam
