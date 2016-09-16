#include "fcl-extern.hpp"

namespace fcl {
//==============================================================================
template
std::size_t collide(
    const CollisionObject<float>* o1,
    const CollisionObject<float>* o2,
    const CollisionRequest<float>& request,
    CollisionResult<float>& result);

//==============================================================================
template
std::size_t collide(
    const CollisionGeometry<float>* o1,
    const Transform3<float>& tf1,
    const CollisionGeometry<float>* o2,
    const Transform3<float>& tf2,
    const CollisionRequest<float>& request,
    CollisionResult<float>& result);
}  // namespace fcl