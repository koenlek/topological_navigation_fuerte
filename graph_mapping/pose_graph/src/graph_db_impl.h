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
 * Template member implementations for graph db
 *
 * \author Bhaskara Marthi
 */

#ifndef POSE_GRAPH_GRAPH_DB_IMPL_H
#define POSE_GRAPH_GRAPH_DB_IMPL_H

#include <pose_graph/exception.h>
#include <graph_mapping_utils/general.h>
#include <ros/ros.h>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>


namespace pose_graph
{

namespace util=graph_mapping_utils;
namespace mr=mongo_ros;

using std::string;
using std::map;
using std::vector;
using boost::format;

typedef boost::mutex::scoped_lock Lock;

template <class M>
typename M::ConstPtr
getNodeData (mr::MessageCollection<M>* client, const unsigned n)
{
  mr::Query q;
  q.append("node", n);
  vector<typename mr::MessageWithMetadata<M>::ConstPtr> res =
    client->pullAllResults(q);
  if (res.size() != 1)
    throw DataNotFoundException(n, res.size());
  return res[0];
}


/*
template <class M>
CachedNodeMap<M>::CachedNodeMap (mr::MessageCollection<M>* coll) :
  initialized_(false),
  coll_(coll),
  insert_sub_(nh_.subscribe("warehouse/" + client->dbName() + "/" + name +
                            "/notify", 100, &CachedNodeMap::updateCB, this))
{
  initialLoad();
  initialized_ = true;
}

template <class M>
CachedNodeMap<M>::CachedNodeMap (wh::WarehouseClient* client,
                                 const string& name) :
  initialized_(false),
  coll_ptr_(new wh::Collection<M>(client->setupCollection<M>
                                  (name, nodeIndex(), nodeIndex()))),
  coll_(coll_ptr_.get()),
  insert_sub_(nh_.subscribe("warehouse/" + client->dbName() + "/" + name +
                            "/notify", 100, &CachedNodeMap::updateCB, this))
{
  initialLoad();
  initialized_ = true;
  }*/


template <class M>
CachedNodeMap<M>::CachedNodeMap (const string& db, const string& coll) :
  initialized_(false),
  coll_(new mr::MessageCollection<M>(db, coll))
{
  coll_->ensureIndex("node");
  initialLoad();
  initialized_ = true;
}



template <class M>
CachedNodeMap<M>::CachedNodeMap (const CachedNodeMap<M>& m) :
  initialized_(m.initialized_),
  coll_(m.coll_), map_(m.map_)
{
  ROS_ASSERT(initialized_);
}

template <class M>
void CachedNodeMap<M>::initialLoad () 
{
  map<unsigned, typename M::ConstPtr> map;
  BOOST_FOREACH (typename mr::MessageWithMetadata<M>::ConstPtr res,
                 coll_->pullAllResults(mr::Query()))
  {
    const unsigned n = res->lookupInt("node");
    map[n] = res;
    ROS_DEBUG_STREAM_NAMED ("cached_node_map", "Preloaded data for " << n);
  }
  Lock l(mutex_);
  map_ = map;
}

template <class M>
typename M::ConstPtr
CachedNodeMap<M>::get (const unsigned n) const
{
  typedef map<unsigned, typename M::ConstPtr> Map;
  typename Map::const_iterator pos;
  {
    Lock l(mutex_);
    pos = map_.find(n);
  }
  if (pos == map_.end()) {
    ROS_ERROR_STREAM_NAMED ("cached_node_map", "Getting data for node " << n <<
                            " from collection ");
    typename M::ConstPtr msg = getNodeData(coll_.get(), n);
    Lock l(mutex_);
    return (map_[n] = msg);
  }
  else
    return pos->second;
}

template <class M>
void CachedNodeMap<M>::removeExisting (const unsigned n)
{
  mr::Query q;
  q.append("node", n);
  const unsigned nr = coll_->removeMessages(q);
  ROS_WARN_COND (nr>1, "Removed %u messages corresponding to node %u", nr, n);
  Lock l(mutex_);
  map_.erase(n);
}

template <class M>
void
CachedNodeMap<M>::set (const unsigned n, typename M::ConstPtr msg)
{
  removeExisting(n);
  coll_->insert(*msg, mr::Metadata().append("node", n));
  Lock l(mutex_);
  map_[n] = msg;
}

template <class M>
void
CachedNodeMap<M>::set (const unsigned n, const M& msg)
{
  typename M::ConstPtr m(new M(msg));
  set(n, m);
}

template <class M>
bool CachedNodeMap<M>::hasData (const unsigned n) const
{
  {
    Lock l(mutex_);
    if (util::contains(map_, n))
      return true;
  }
  typename mr::QueryResults<M>::range_t res =
    coll_->queryResults(mr::Query().append("node", n), true);
  return (!(res.first == res.second));
}




} // namespace

#endif // include guard
