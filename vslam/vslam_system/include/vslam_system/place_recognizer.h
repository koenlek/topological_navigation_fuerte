/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef VSLAM_SYSTEM_PLACE_RECOGNIZER_H
#define VSLAM_SYSTEM_PLACE_RECOGNIZER_H

#include <frame_common/frame.h>
#include <vocabulary_tree/generic_tree.h>
#include <vocabulary_tree/database.h>

namespace vslam {

typedef std::vector<frame_common::Frame, Eigen::aligned_allocator<frame_common::Frame> > FrameVector;

/**
 * \brief Recognizes previously seen frames.
 */
class PlaceRecognizer
{
public:
  /**
   * \brief Constructor
   *
   * Can use:
   * PlaceRecognizer("/u/mihelich/vocab/holidays.tree", "/u/mihelich/vocab/holidays.weights")
   *
   * \param tree_file    The file containing the vocabulary words
   * \param weights_file The file containing the weights
   */
  PlaceRecognizer(const std::string& tree_file,
                  const std::string& weights_file);

  /**
   * \brief Insert a new frame with the provided id.
   *
   * \param frame The frame to insert
   * \param id    The frame id
   */
  void insert(const frame_common::Frame& frame, uint32_t id);

  /**
   * \brief Find the top N matches for the query frame, and also insert it
   * for future recognition.
   *
   * \param      frame      The query frame, also to be inserted
   * \param      id         The id to assign to the query frame
   * \param      all_frames Collection of frames indexable by the saved ids
   * \param      N          The number of matches
   * \param[out] matches    The top N matching frames
   */
  void findAndInsert(const frame_common::Frame& frame, uint32_t id,
                     const FrameVector& all_frames, size_t N,
                     std::vector<const frame_common::Frame*>& matches);

private:
  vt::GenericTree tree_;
  vt::Database database_;
  std::vector<uint32_t> user_ids_;
};

} //namespace vslam

#endif
