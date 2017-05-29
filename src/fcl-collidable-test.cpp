/*
 * fcl-collidable-test.cpp
 * Copyright (C) 2017 rdominguez <rdominguez@RDOMINGUEZ>
 *
 * Distributed under terms of the MIT license.
 */

#include <QObject>
#include "Collision.hpp"

#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>

#include <smurf/Robot.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_visualizer/EnvireVisualizerWindow.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Transform.hpp>

#include <envire_smurf/GraphLoader.hpp>

#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <plugin_manager/PluginLoader.hpp>

#include <envire_core/graph/GraphViz.hpp>

#include <Eigen/Geometry>

#define CENTER_FRAME_NAME std::string("center")
#define MLS_FRAME_NAME std::string("MLS Frame")

using namespace envire::viz;

struct EdgeCallBack : public envire::core::GraphEventDispatcher
{
    const envire::core::EnvireGraph &graph_;
    const smurf::Collidable& collidable_;
public:
    EdgeCallBack(const envire::core::EnvireGraph& graph, const smurf::Collidable & collidable) : graph_(graph), collidable_(collidable) {}
    void edgeModified(const envire::core::EdgeModifiedEvent& e) {
        std::cout << "Edge modified. Trafo=\n";
        std::cout << graph_.getEdgeProperty(e.edge).transform.getTransform().matrix();
        fcl::Transform3f trafo = graph_.getEdgeProperty(e.edge).transform.getTransform().cast<float>();
        fcl::CollisionRequestf request(10, true, 10, true);
        fcl::CollisionResultf result;
        //fcl::Spheref sphere(.25);
        fcl::Boxf box(1.0, 1.0, 1.0);
        fcl::collide_collidable(collidable_, trafo, &box, request, result);
        std::cout << "\nisCollision()==" << result.isCollision();
        for(size_t i=0; i< result.numContacts(); ++i)
        {
            const auto & cont = result.getContact(i);
            std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
        }
        std::cout << std::endl;
    }
};


int main(int argc, char **argv) {

    // Use smurf_loader to load an smurf file
    //const std::string path="./smurf/just_a_box/smurf/just_a_box.smurf";
    //const std::string path="./smurf/two_boxes_joined/smurf/two_boxes.smurf";
    const std::string path="./smurf/two_boxes_joined/smurf/two_spheres.smurf";
    //const std::string path="./smurf/asguard_v4/smurf/asguard_v4.smurf";
    smurf::Robot* robot = new(smurf::Robot);
    robot->loadFromSmurf(path);
    envire::core::GraphViz viz;

    envire::core::Transform iniPose;
    std::shared_ptr<envire::core::EnvireGraph> graph(new envire::core::EnvireGraph) ;
    iniPose.transform.orientation = base::Quaterniond::Identity();
    iniPose.transform.translation << 1.0, 1.0, 1.0;
    envire::smurf::GraphLoader graphLoader(graph, iniPose);
    envire::core::FrameId center = CENTER_FRAME_NAME;
    graph->addFrame(center);
    graphLoader.loadStructure(graph->getVertex(center), *robot);
    
    int nextGroupId = 0;
    graphLoader.loadFrames(nextGroupId, *robot);
    graphLoader.loadCollidables(*robot);
    //graphLoader.loadVisuals(*robot);
    viz.write(*(graphLoader.getGraph()), "fcl-collidable-test.dot");
    
    // Load the mls
    if(argc < 3)
    {
        std::cout << "Usage:\n" << argv[0] << " mlsFileName " << "sloped|kalman \n";
        return 0;
    }

    plugin_manager::PluginLoader* loader = plugin_manager::PluginLoader::getInstance();
    envire::core::ItemBase::Ptr item;
    std::string mapType = argv[2];
    if (mapType == "sloped")
    {
        loader->createInstance("envire::core::Item<maps::grid::MLSMapSloped>", item);
    }
    else
    {
        std::cout << "Only deserialization of sloped mls is supported";
    }

    envire::core::Item<MLSMapS>::Ptr mlsItem = boost::dynamic_pointer_cast<envire::core::Item<MLSMapS>>(item);

    std::ifstream fileIn(argv[1]);

    // deserialize from file stream
    boost::archive::binary_iarchive mlsIn(fileIn);

    mlsIn >> mlsItem->getData();

#if 0
    // hard-coded testing:
    std::shared_ptr<fcl::Spheref> geo1(new fcl::Spheref(.25));

    fcl::CollisionRequestf request(10, true, 10, true);
    fcl::CollisionResultf result;


    fcl::Transform3f trafo;
    trafo.setIdentity();
    for(float z=-4.f; z<=4.f; z+=0.125f )
    {
        result.clear(); // Must clear result before calling fcl::collide functions!!!
        trafo.translation().z() = z;
        fcl::collide_mls(mlsItem->getData(), trafo, geo1.get(), request, result);
        std::cout << "With z = " << z << ", isCollision()==" << result.isCollision();
        for(size_t i=0; i< result.numContacts(); ++i)
        {
            const auto & cont = result.getContact(i);
            std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
        }
        std::cout << std::endl;

    }
#endif

    QApplication app(argc, argv);
    EnvireVisualizerWindow window;
    //std::shared_ptr<envire::core::EnvireGraph> graph(new envire::core::EnvireGraph);
    //graph->addFrame("A");
    graph->addFrame(MLS_FRAME_NAME);
    graph->addItemToFrame(MLS_FRAME_NAME, mlsItem);
    envire::core::Transform mlsCenter(base::Position(1, 1, 1), Eigen::Quaterniond::Identity());
    graph->addTransform(MLS_FRAME_NAME, CENTER_FRAME_NAME, mlsCenter);
    QString qCentreName = QString::fromStdString(CENTER_FRAME_NAME);
    window.displayGraph(graph, qCentreName);
    window.show();

    //graph->subscribe(new EdgeCallBack(*graph, mlsItem->getData()));

    app.exec();
}
