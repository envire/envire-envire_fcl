//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

#include "fcl-extern.hpp"

#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>

#include <Eigen/Geometry>



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
    // m1.bv_splitter.reset(new fcl::detail::BVSplitter<BV>(fcl::detail::SPLIT_METHOD_MEDIAN);
    //estimate number of cells:
    size_t num_cells = ((bv2.max_ - bv2.min_).template head<2>().cwiseQuotient(mls.getResolution().cast<S>())).prod();
    Eigen::Vector2f cell_size = mls.getResolution().cast<float>();
    std::cout << "num_cells: " << num_cells << ", cell_size: " << cell_size.transpose() << "\n";
    m1.beginModel(2*num_cells, 4*num_cells); // assume that each cell creates one 4-gon on average
    Eigen::AlignedBox3d bounding(bv2.min_.template cast<double>(), bv2.max_.template cast<double>());
    mls.intersectAABB_callback(bounding,
            [&m1, &cell_size](maps::grid::Index idx, const P& p) {
//        std::cout << "Intersect at " << idx.transpose() << " with p from " << p.getMin() << " to " << p.getMax() << "\t";
        std::vector<Eigen::Vector3f> points;
        maps::grid::getPolygon(points, p, idx, cell_size);
//        std::cout << "Polygon:";
//        for(auto pt : points) std::cout << " [" << pt.transpose() << "]";
//        std::cout << std::endl;
//        if(points.size() < 3) return;
//        if(points.size() > 6) std::cout << "WHAT? " << p.getNormal().transpose() << "\n\n";
        assert(points.size()>=3 && points.size() <=6);
        m1.addSubModel(points, TriVect(triags, triags+(points.size()-2)));
    });
    if(m1.num_vertices==0)
    {
        // no contacts at all, m1.endModel() would fail, so we directly return here
        return;
    }
    m1.endModel();
    fcl::collide(&m1, world2map.inverse(Eigen::Isometry), o2, tf2, request, result);
}


}  // namespace fcl


int main(int argc, char **argv) {

    if(argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " mlsFileName\n";
        return 0;
    }

    std::ifstream fileIn(argv[1]);

    // deserialize from file stream
    boost::archive::binary_iarchive mlsIn(fileIn);
    std::shared_ptr<MLSMapS> mlsSloped(new MLSMapS);

    mlsIn >> *mlsSloped;
    std::shared_ptr<fcl::Spheref> geo1(new fcl::Spheref(.251));

    fcl::CollisionRequestf request(10, true, 10, true);
    fcl::CollisionResultf result;


    fcl::Transform3f trafo;
    trafo.setIdentity();
    for(float z=-4.f; z<=4.f; z+=0.125f )
    {
        result.clear(); // Must clear result before calling fcl::collide functions!!!
        trafo.translation().z() = z;
        fcl::collide_mls(*mlsSloped, trafo, geo1.get(), request, result);
        //    fcl::collide(&o1, &o2, request, result);
        std::cout << "With z = " << z << ", isCollision()==" << result.isCollision();
        for(size_t i=0; i< result.numContacts(); ++i)
        {
            const auto & cont = result.getContact(i);
            std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
        }
        std::cout << std::endl;

    }





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
