/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * Utility code for using the Warehouse to store info about the graph
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_GRAPH_DB_H
#define POSE_GRAPH_GRAPH_DB_H

#include <pose_graph/constraint_graph.h>
#include <mongo_ros/message_collection.h>
#include <std_msgs/String.h>

namespace pose_graph
{

/// \brief Get the message associated with node \a n of the graph
/// \throws DataNotFoundException
template <class M>
typename M::ConstPtr
getNodeData (mongo_ros::MessageCollection<M>* coll, unsigned n);



/// \brief A map<unsigned, M::ConstPtr> backed by a db collection
///
/// Can have only one of these per collection per ros node
template <class M>
class CachedNodeMap
{
public:

  CachedNodeMap (const std::string& db, const std::string& coll);

  /// \brief Copy constructor
  CachedNodeMap (const CachedNodeMap& m);

  /// \brief Get the message associated with this node.  Calls out to the
  /// warehouse only if the message is not found in the cache.
  typename M::ConstPtr get (unsigned n) const;

  /// \brief Set the message associated with this node and insert it
  /// to the db.   Remove any existing message.
  void set (unsigned n, typename M::ConstPtr);

  /// \brief Set the message associated with this node and publish it
  /// to the warehouse.  Remove any existing message.
  void set (unsigned n, const M& m);

  /// \brief Does this node have associated node data?  Calls out
  /// to the warehouse unless the data is present in the cache.
  bool hasData (unsigned n) const;

private:

  /// Load all messages initially present
  void initialLoad ();

  /// Remove existing messages for a given node
  void removeExisting (unsigned n);

  // Mutable since const ops can acquire the lock
  mutable boost::mutex mutex_;

  // True if constructor has finished running
  bool initialized_;

  boost::shared_ptr<mongo_ros::MessageCollection<M> > coll_;

  // Mutable since it's used as a cache by const ops
  mutable std::map<unsigned, typename M::ConstPtr> map_;

  ros::NodeHandle nh_;
  
};



} // namespace

#include "../../src/graph_db_impl.h"

#endif // include guard
