#include "fcl-extern.hpp"

#include <maps/grid/MLSMap.hpp>
#include <smurf/Collidable.hpp>

typedef std::shared_ptr<fcl::CollisionGeometry<float> > GeoPtr;
typedef maps::grid::MLSMapSloped MLSMapS;

namespace fcl {

template<class Shape, class S>
void collide_mls(
        const maps::grid::MLSMapSloped& mls,
        const Transform3<S>& tf2,
        const Shape* o2,
        const CollisionRequest<S>& request,
        CollisionResult<S>& result
)
{
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
}

// Do not recompile this function every time:

extern template void collide_mls(
        const maps::grid::MLSMapSloped& mls,
        const Transform3<float>& tf2,
        const fcl::Spheref* o2,
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
