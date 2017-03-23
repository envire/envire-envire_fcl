//
// Copyright (c) 2015, Deutsches Forschungszentrum für Künstliche Intelligenz GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#pragma GCC diagnostic push
// disable 'has virtual functions and accessible non-virtual destructor' warnings inside fcl:
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#include <fcl/fcl.h>
#pragma GCC diagnostic pop

// Declare some templates as external, so they don't have to be re-compiled every time
// FCL itself does this only for double types, but float is better for MLS interaction

namespace fcl {

extern template
class CollisionGeometry<float>;
extern template
class CollisionObject<float>;
extern template
struct CollisionRequest<float>;
extern template
struct CollisionResult<float>;


extern template
std::size_t collide(
    const CollisionObject<float>* o1,
    const CollisionObject<float>* o2,
    const CollisionRequest<float>& request,
    CollisionResult<float>& result);

//==============================================================================
extern template
std::size_t collide(
    const CollisionGeometry<float>* o1,
    const Transform3<float>& tf1,
    const CollisionGeometry<float>* o2,
    const Transform3<float>& tf2,
    const CollisionRequest<float>& request,
    CollisionResult<float>& result);

}  // namespace fcl
