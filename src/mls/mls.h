/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2016, University of Bremen
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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
 */

/** @author Christoph Hertzberg */

#ifndef FCL_MLS_H
#define FCL_MLS_H

#include "fcl/config.h"

#include <memory>
#include <array>

#include <maps/grid/MLSMap.hpp>

#include "fcl/math/bv/AABB.h"
#include "fcl/narrowphase/collision_object.h"

namespace fcl
{

class MLSMap : public CollisionGeometry<double>
{
private:
  std::shared_ptr<const maps::grid::MLSMapSloped> mls;

public:

  /// @brief construct MLSMap from MLSMapSloped
  MLSMap(const std::shared_ptr<const maps::grid::MLSMapSloped>& mls_) : mls(mls_) {}

  /// @brief compute the AABB<S> of the MLS in its local coordinate system
  void computeLocalAABB();

  /// @brief return object type, it is an octree
  OBJECT_TYPE getObjectType() const { return OT_UNKNOWN; }

  /// @brief return node type, it is an octree
  NODE_TYPE getNodeType() const { return BV_UNKNOWN; }

  /// @brief getter for the internal MLS
  const maps::grid::MLSMapSloped& getMLS() const { return *mls; }

};

} // namespace fcl

#include "mls-inl.h"

#endif
