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
 * General purpose utilities
 *
 * \author Bhaskara Marthi
 */

#ifndef GRAPH_MAPPING_UTILS_GENERAL_H
#define GRAPH_MAPPING_UTILS_GENERAL_H

#include <ros/assert.h>
#include <cstdlib>
#include <boost/foreach.hpp>
#include <map>
#include <set>
#include <stdexcept>

namespace graph_mapping_utils
{

// Check if a container contains a key
template <class Key, class Container>
bool contains (const Container& container, const Key& key)
{
  return container.find(key)!=container.end();
}


// Get value corresponding to a key in a map, while asserting if it's not there
template <class K, class V, class C, class A>
const V& keyValue (const std::map<K, V, C, A>& m, const K& key)
{
  typename std::map<K, V, C, A>::const_iterator pos = m.find(key);
  if (pos==m.end())
    throw std::out_of_range("Map did not contain key it was expected to");
  return pos->second;
}

/// \brief Sample a subset of set \a s of size \a n
/// \param n the size of the subset.  If this is greater than the size of s,
/// or less than 0, then we just return s.
template <class T>
std::set<T> sampleSubset (const std::set<T>& s, const int n)
{
  if (n>(int)s.size() || n < 0)
    return s;
  else {
    std::set<T> s2;
    unsigned elements_left_to_sample = s.size();
    unsigned elements_needed = n;
    BOOST_FOREACH (const T& x, s) {
      const double p = (double) elements_needed/elements_left_to_sample;
      const double q = (double)rand()/(double)RAND_MAX;
      if (q<p) {
        s2.insert(x);
        elements_needed--;
      }
      elements_left_to_sample--;      
    }
    return s2;
  }
}

} // namespace

#endif // include guard
