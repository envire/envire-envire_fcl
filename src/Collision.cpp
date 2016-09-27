#include "Collision.hpp"


namespace fcl {

template void collide_mls(
        const maps::grid::MLSMapSloped& mls,
        const Transform3<float>& tf2,
        const fcl::Spheref* o2,
        const CollisionRequest<float>& request,
        CollisionResult<float>& result
);

}  // namespace fcl
