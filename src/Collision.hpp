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

#include "fcl-extern.hpp"

#include <maps/grid/MLSMap.hpp>
#include <smurf/Collidable.hpp>

//#define DEBUG 1

typedef std::shared_ptr<fcl::CollisionGeometry<float> > GeoPtr;
typedef maps::grid::MLSMapSloped MLSMapS;
typedef maps::grid::MLSMapPrecalculated MLSMapP;

namespace fcl {

template<class Shape, class S, enum maps::grid::MLSConfig::update_model SurfaceType>
void collide_mls(
        const maps::grid::MLSMap<SurfaceType>& mls,
        const Transform3<S>& tf2,
        const Shape* o2,
        const CollisionRequest<S>& request,
        CollisionResult<S>& result
)
{
    // FIXME Local frame of MLS is slightly annoying here ...
    const Transform3<S> world2map = mls.getLocalFrame().template cast<S>();
    const Transform3<S> shape2map = world2map * tf2;

    // compute bounding volume of o2 relative to mls map
    AABB<S> bv2;
    computeBV(*o2, shape2map, bv2);
#ifdef DEBUG
    std::cout << "[envire_fcl::collide_mls] world2map:\n" << world2map.matrix() << "\nshape2map:\n" << shape2map.matrix() << "\nBV: [" << bv2.min_.transpose() << "] - [" << bv2.max_.transpose() << "]" << std::endl;
#endif

    typedef fcl::AABB<S> BV;
    typedef typename maps::grid::MLSMap<SurfaceType>::Patch P;

    typedef std::vector<fcl::Triangle> TriVect;
    static fcl::Triangle triags[4];
    for(int i=0; i<4; ++i) triags[i].set(0, i+1, i+2);
    fcl::BVHModel<BV> m1;  // BVHModel of local MLS region
    // TODO check out if different a bv_splitter makes a difference:
    // m1.bv_splitter.reset(new fcl::detail::BVSplitter<BV>(fcl::detail::SPLIT_METHOD_MEDIAN);
    //estimate number of cells:
    size_t num_cells = ((bv2.max_ - bv2.min_).template head<2>().cwiseQuotient(mls.getResolution().template cast<S>()).array().ceil()).prod();
    Eigen::Vector2f cell_size = mls.getResolution().template cast<float>();
#ifdef DEBUG
    std::cout << "[envire_fcl::collide_mls] world2map:num_cells: " << num_cells << ", cell_size: " << cell_size.transpose() << std::endl;

#endif
    m1.beginModel(2*num_cells, 4*num_cells); // assume that each cell creates one 4-gon on average
    Eigen::AlignedBox3d bounding(bv2.min_.template cast<double>(), bv2.max_.template cast<double>());
    mls.intersectAABB_callback(bounding,
            [&m1, &cell_size](maps::grid::Index idx, const P& p) {
        std::vector<Eigen::Vector3f> points;
        maps::grid::getPolygon(points, p, idx, cell_size);
        if(points.size() < 3)
        {
            std::cerr << "[envire_fcl::collide_mls] No intersection of patch with box\n";
            return false;
        }
        assert(points.size()>=3 && points.size() <=6);
        m1.addSubModel(points, TriVect(triags, triags+(points.size()-2)));
        // return false, because we need to add every possible intersection:
        return false;
    });
    if(m1.num_vertices==0)
    {
        // no contacts at all, m1.endModel() would fail, so we directly return here
        return;
    }
    m1.endModel();
#ifdef DEBUG
    std::cout << "[envire_fcl::collide_mls] Number of triangles: " << m1.num_tris << std::endl;
#endif
    // perform actual fcl-collision test:
    fcl::collide(&m1, world2map.inverse(Eigen::Isometry), o2, tf2, request, result);
}

// Do not recompile these functions every time:

extern template void collide_mls(
        const maps::grid::MLSMapSloped& mls,
        const Transform3<float>& tf2,
        const fcl::Spheref* o2,
        const CollisionRequest<float>& request,
        CollisionResult<float>& result
);

extern template void collide_mls(
        const maps::grid::MLSMapSloped& mls,
        const Transform3<float>& tf2,
        const fcl::Boxf* o2,
        const CollisionRequest<float>& request,
        CollisionResult<float>& result
);

extern template void collide_mls(
        const maps::grid::MLSMapPrecalculated& mls,
        const Transform3<float>& tf2,
        const fcl::Spheref* o2,
        const CollisionRequest<float>& request,
        CollisionResult<float>& result
);

extern template void collide_mls(
        const maps::grid::MLSMapPrecalculated& mls,
        const Transform3<float>& tf2,
        const fcl::Boxf* o2,
        const CollisionRequest<float>& request,
        CollisionResult<float>& result
);


template<class Shape, class S>
void collide_collidable(
        const smurf::Collidable& collidable,
        const Transform3<S>& tf2,
        const Shape* o2,
        const CollisionRequest<S>& request,
        CollisionResult<S>& result
)
{
    /*
    // FIXME Local frame of MLS is slightly annoying here ...
    const Transform3<S> world2map = mls.getLocalFrame().cast<S>();
    const Transform3<S> shape2map = world2map * tf2;

    // compute bounding volume of o2 relative to mls map
    AABB<S> bv2;
    computeBV(*o2, shape2map, bv2);
    std::cout << "world2map:\n" << world2map.matrix() << "\nshape2map:\n" << shape2map.matrix() << "\nBV: [" << bv2.min_.transpose() << "] - [" << bv2.max_.transpose() << "]\n";

    typedef fcl::AABB<S> BV;
    typedef maps::grid::MLSMapSloped::Patch P;

    typedef std::vector<fcl::Triangle> TriVect;
    static fcl::Triangle triags[4];
    for(int i=0; i<4; ++i) triags[i].set(0, i+1, i+2);
    fcl::BVHModel<BV> m1;  // BVHModel of local MLS region
    // TODO check out if different a bv_splitter makes a difference:
    // m1.bv_splitter.reset(new fcl::detail::BVSplitter<BV>(fcl::detail::SPLIT_METHOD_MEDIAN);
    //estimate number of cells:
    size_t num_cells = ((bv2.max_ - bv2.min_).template head<2>().cwiseQuotient(mls.getResolution().cast<S>())).prod();
    Eigen::Vector2f cell_size = mls.getResolution().cast<float>();
    std::cout << "num_cells: " << num_cells << ", cell_size: " << cell_size.transpose() << "\n";
    m1.beginModel(2*num_cells, 4*num_cells); // assume that each cell creates one 4-gon on average
    Eigen::AlignedBox3d bounding(bv2.min_.template cast<double>(), bv2.max_.template cast<double>());
    mls.intersectAABB_callback(bounding,
            [&m1, &cell_size](maps::grid::Index idx, const P& p) {
        std::vector<Eigen::Vector3f> points;
        maps::grid::getPolygon(points, p, idx, cell_size);
        assert(points.size()>=3 && points.size() <=6);
        m1.addSubModel(points, TriVect(triags, triags+(points.size()-2)));
        // return false, because we need to add every possible intersection:
        return false;
    });
    if(m1.num_vertices==0)
    {
        // no contacts at all, m1.endModel() would fail, so we directly return here
        return;
    }
    m1.endModel();
    std::cout << "Number of triangles: " << m1.num_tris << "\n";
    // perform actual fcl-collision test:
    fcl::collide(&m1, world2map.inverse(Eigen::Isometry), o2, tf2, request, result);
    */
}

}  // namespace fcl
