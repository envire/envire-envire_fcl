//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

#include <fcl/fcl.h>
#include <envire_fcl/mls/mls.h>

#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>

#include <Eigen/Geometry>



struct bla : public fcl::Sphere<double>
{
    bla(double radius) : Sphere(radius) {}
//    fcl::NODE_TYPE getNodeType() const { return fcl::BV_UNKNOWN; }
};

typedef std::shared_ptr<fcl::CollisionGeometry<float> > GeoPtr;
typedef maps::grid::MLSMapSloped MLSMapS;

typedef fcl::detail::GJKSolver_libccd<double> NarrowPhaseSolver;
namespace fcl {
// extern template to avoid compiling this every time:
extern template detail::CollisionFunctionMatrix<NarrowPhaseSolver>& getCollisionFunctionLookTable< NarrowPhaseSolver>();

#if 0
// obsolete

template <typename NarrowPhaseSolver>
class MLSSolver {

    using S = typename NarrowPhaseSolver::S;

    const NarrowPhaseSolver* solver;

    mutable const CollisionRequest<S>* crequest;
    mutable const DistanceRequest<S>* drequest;

    mutable CollisionResult<S>* cresult;
    mutable DistanceResult<S>* dresult;

protected:
    template<typename Shape>
    void patch_shape_intersect();

public:
    MLSSolver(const NarrowPhaseSolver * s)
    : solver(s)
    , crequest(nullptr), drequest(nullptr)
    , cresult(nullptr), dresult(nullptr)
    {}

    template <typename Shape>
    void shapeIntersect(const MLSMap* map, const Shape& s,
                        const Transform3<S>& tf1, const Transform3<S>& tf2,
                        const CollisionRequest<S>& request_,
                        CollisionResult<S>& result_) const
    {
        const MLSMapS & mls = *(map->mls);
        // FIXME Local frame of MLS is slightly annoying here ...
        const Transform3<S> map2world = tf1 * mls.getLocalFrame().cast<S>();
        const Transform3<S> shape2map = map2world.inverse(Eigen::Isometry) * tf2;

        AABB<S> bv2;
        computeBV(s, shape2map, bv2);

        mls.intersectAABB_callback(bv2);



    }


};






template <typename Shape, typename NarrowPhaseSolver>
class MLSShapeCollisionTraversalNode
    : public detail::CollisionTraversalNodeBase<typename Shape::S>
{

};



template<class Shape>
std::size_t collide_mls_shape(
      const CollisionGeometry<typename Shape::S>* o1,
      const Transform3<typename Shape::S>& tf1,
      const CollisionGeometry<typename Shape::S>* o2,
      const Transform3<typename Shape::S>& tf2,
      const NarrowPhaseSolver* nsolver,
      const CollisionRequest<typename Shape::S>& request,
            CollisionResult<typename Shape::S>& result)
{
    typedef typename Shape::S S;
    const MLSMap* obj1 = static_cast<const MLSMap*>(o1);
    const Shape* obj2 = static_cast<const Shape*>(o2);

    return 0;
}


#endif


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
        std::cout << "Intersect at " << idx.transpose() << " with p from " << p.getMin() << " to " << p.getMax() << "\t";
        std::vector<Eigen::Vector3f> points;
        maps::grid::getPolygon(points, p, idx, cell_size);
        std::cout << "Polygon:";
        for(auto pt : points) std::cout << " [" << pt.transpose() << "]";
        std::cout << std::endl;
        if(points.size() < 3) return;
        if(points.size() > 6) std::cout << "WHAT? " << p.getNormal().transpose() << "\n\n";
        assert(points.size()>=3);
        m1.addSubModel(points, TriVect(triags, triags+(points.size()-2)));
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


int main(int argc, char **argv) {

    if(argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " mlsFileName\n";
        return 0;
    }

    std::ifstream fileIn(argv[1]);

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive mlsIn(fileIn);
    std::shared_ptr<MLSMapS> mlsSloped(new MLSMapS);

    mlsIn >> *mlsSloped;
    std::shared_ptr<fcl::Spheref> geo1(new fcl::Spheref(5));
//    fcl::MLSMap *mls = new fcl::MLSMap(mlsSloped);
//    GeoPtr mls_g(mls);
//    fcl::BVHModel
//    fcl::computeBV()

//    fcl::CollisionObjectd o1(mls_g, fcl::Transform3d(Eigen::Translation3d(Eigen::Vector3d::UnitX())));
//    fcl::CollisionObjectd o2(geo1, fcl::Transform3d(Eigen::Translation3d(Eigen::Vector3d::UnitY())));
//    auto col_mat = fcl::getCollisionFunctionLookTable< NarrowPhaseSolver>().collision_matrix;
//    col_mat[fcl::GEOM_MLS][fcl::GEOM_SPHERE] = &fcl::collide_mls_shape<fcl::Sphere<double> >;
    fcl::CollisionRequestf request(10, true, 10, true);
    fcl::CollisionResultf result;



    fcl::collide_mls(*mlsSloped, fcl::Transform3f::Identity(), geo1.get(), request, result);
//    fcl::collide(&o1, &o2, request, result);
    std::cout << result.isCollision();
    for(size_t i=0; i< result.numContacts(); ++i)
    {
        const auto & cont = result.getContact(i);
        std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
    }
    std::cout << std::endl;





    QtThreadedWidget<vizkit3d::Vizkit3DWidget> app;
    app.start();
    //QApplication app(argc, argv);
    vizkit3d::Vizkit3DWidget *widget = app.getWidget();
    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);

    vizkit3d::MLSMapVisualization mlsViz;
    widget->addPlugin(&mlsViz);
    mlsViz.updateData(*mlsSloped);
    mlsViz.setPluginEnabled(!false);
    mlsViz.setShowPatchExtents(false);

    widget->show();




    while(app.isRunning() && (!usleep(1000)))
    {
        //
    }


}
