//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

#include <fcl/fcl.h>
#include <envire_fcl/mls/mls.h>

#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>

#include <Eigen/Geometry>



#if 0

typedef std::shared_ptr<fcl::CollisionGeometry<float> > GeoPtr;
typedef maps::grid::MLSMapSloped MLSMapS;

typedef fcl::detail::GJKSolver_libccd<double> NarrowPhaseSolver;
namespace fcl {
// extern template to avoid compiling this every time:
extern template detail::CollisionFunctionMatrix<NarrowPhaseSolver>& getCollisionFunctionLookTable< NarrowPhaseSolver>();



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
    std::cout << "map2world:\n" << world2map.matrix() << "\nshape2map:\n" << shape2map.matrix() << "\nBV: [" << bv2.min_.transpose() << "] - [" << bv2.max_.transpose() << "]\n";

    typedef fcl::AABB<S> BV;
    typedef maps::grid::MLSMapSloped::Patch P;

    typedef std::vector<fcl::Triangle> TriVect;
    static fcl::Triangle triags[4];
    for(int i=0; i<4; ++i) triags[i].set(0, i+1, i+2);
    fcl::BVHModel<BV> m1;  // BVHModel of local MLS region
    // m1.bv_splitter.reset(new fcl::detail::BVSplitter<BV>(fcl::detail::SPLIT_METHOD_MEDIAN);
    //estimate number of cells:
    size_t num_cells = ((bv2.max_ - bv2.min_).template head<2>().cwiseQuotient(mls.getResolution().cast<S>())).prod();
    Eigen::Vector2f cell_size = mls.getResolution().cast<float>();
    std::cout << "num_cells: " << num_cells << ", cell_size: " << cell_size.transpose() << "\n";
    m1.beginModel(2*num_cells, 4*num_cells); // assume that each cell creates one 4-gon on average
    Eigen::AlignedBox3d bounding(bv2.min_.template cast<double>(), bv2.max_.template cast<double>());
    mls.intersectAABB_callback(bounding,
            [&m1, &cell_size](maps::grid::Index idx, const P& p) {
        std::cout << "Intersect at " << idx.transpose() << " with p from " << p.getMin() << " to " << p.getMax() << "\n";
        std::vector<Eigen::Vector3f> points;
        maps::grid::getPolygon(points, p, idx, cell_size);
        m1.addSubModel(points, TriVect(triags, triags+points.size()+3));
    });
    if(m1.num_vertices==0)
    {
        // no contacts at all
        result.clear();
        return;
    }
    m1.endModel();
    fcl::collide(&m1, Transform3<S>::Identity(), o2, shape2map, request, result);
}


}  // namespace fcl
#endif

int main(int argc, char **argv) {


    typedef float S;
    typedef fcl::AABB<S> BV;
    std::shared_ptr<fcl::Spheref> geo1(new fcl::Spheref(20));


    fcl::CollisionRequestf request(100, true, 100, true);
    fcl::CollisionResultf result;


    fcl::BVHModel<BV> m1;  // BVHModel of local MLS region
    // m1.bv_splitter.reset(new fcl::detail::BVSplitter<BV>(fcl::detail::SPLIT_METHOD_MEDIAN);

    typedef Eigen::Vector3f Vec3;
    m1.beginModel(4, 4); // assume that each cell creates one 4-gon on average
    for(float f=0.0f; f<100; f+=1.0)
    {
        std::vector<Vec3> points = {Vec3(0,0,f), Vec3(0,1,f), Vec3(1,0,f), Vec3(1,1,f)};
        std::vector<fcl::Triangle> triags = {fcl::Triangle(0,1,2), fcl::Triangle(0,2,3)};
        m1.addSubModel(points, triags);
    }
    m1.endModel();

    fcl::collide(&m1, fcl::Transform3<S>::Identity(), geo1.get(), fcl::Transform3<S>::Identity(), request, result);


    std::cout << result.isCollision();
    for(size_t i=0; i< result.numContacts(); ++i)
    {
        const auto & cont = result.getContact(i);
        std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
    }
    std::cout << std::endl;

}
