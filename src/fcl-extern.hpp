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
