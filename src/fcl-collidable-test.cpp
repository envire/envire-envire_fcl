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
//#define SMURF_COLLIDABLE_SPHERE_FRAME_NAME std::string("root")
#define SMURF_COLLIDABLE_BOX_FRAME_NAME std::string("box")

using namespace envire::viz;
using CollisionType = smurf::Collidable;
using CollisionItem = envire::core::Item<CollisionType>;
using IterCollItem = envire::core::EnvireGraph::ItemIterator<CollisionItem>;

struct EdgeCallBack : public envire::core::GraphEventDispatcher
{
    const envire::core::EnvireGraph &graph_;
    const envire::core::FrameId & colFrame_;
    const MLSMapS& mls_;

public:
    EdgeCallBack(const envire::core::EnvireGraph& graph, const envire::core::FrameId & colFrame, const MLSMapS & mls) : graph_(graph), colFrame_(colFrame), mls_(mls) {}
    void edgeModified(const envire::core::EdgeModifiedEvent& e) {

        smurf::Collidable collidable;
        IterCollItem itCols, endCols;
        itCols = graph_.getItem<CollisionItem>(colFrame_); 
        collidable = itCols->getData();
        //envire::core::Transform tfColCen = graph_.getTransform(colFrame_, MLS_FRAME_NAME);
        envire::core::Transform tfColCen = graph_.getTransform(MLS_FRAME_NAME, colFrame_);
        fcl::Transform3f trafo = tfColCen.transform.getTransform().cast<float>();
        std::cout << "Edge modified. Trafo=\n";
        std::cout << tfColCen.transform.getTransform().matrix();
        // TODO Here I must give the transformation between the object and the mls frame, currently is only taking the transformation of the edge that just changed. For doing this, you could pass instead of the smurf::Collidable, the frame where it is located, here get the collidable and the correct transformation
        //fcl::Transform3f trafo = graph_.getEdgeProperty(e.edge).transform.getTransform().cast<float>();
        
        fcl::CollisionRequestf request(10, true, 10, true);
        fcl::CollisionResultf result;
        urdf::Collision collision = collidable.getCollision();

        //boost::shared_ptr<urdf::Sphere> sphereUrdf = boost::dynamic_pointer_cast<urdf::Sphere>(collision.geometry);
        //fcl::Spheref sphere(sphereUrdf->radius);

        boost::shared_ptr<urdf::Box> boxUrdf = boost::dynamic_pointer_cast<urdf::Box>(collision.geometry);
        fcl::Boxf box(boxUrdf->dim.x, boxUrdf->dim.y, boxUrdf->dim.z);

        //fcl::Spheref sphere(.25); // Build an sphere out of the collidable
        //fcl::Boxf box(1.0, 1.0, 1.0);

        fcl::collide_mls(mls_, trafo, &box, request, result);
        //fcl::collide_mls(mls_, trafo, &sphere, request, result);

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
    const std::string path="./smurf/just_a_box/smurf/just_a_box.smurf";
    //const std::string path="./smurf/two_boxes_joined/smurf/two_boxes.smurf";
    //const std::string path="./smurf/two_boxes_joined/smurf/two_spheres.smurf";
    //const std::string path="./smurf/two_boxes_joined/smurf/one_sphere.smurf";
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
        envire::core::Item<MLSMapS>::Ptr mlsItem = boost::dynamic_pointer_cast<envire::core::Item<MLSMapS>>(item);
        std::ifstream fileIn(argv[1]);
        // deserialize from file stream
        boost::archive::binary_iarchive mlsIn(fileIn);
        mlsIn >> mlsItem->getData();
        QApplication app(argc, argv);
        EnvireVisualizerWindow window;
        //std::shared_ptr<envire::core::EnvireGraph> graph(new envire::core::EnvireGraph);
        //graph->addFrame("A");
        graph->addFrame(MLS_FRAME_NAME);
        graph->addItemToFrame(MLS_FRAME_NAME, mlsItem);
        envire::core::Transform mlsCenter(base::Position(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
        graph->addTransform(MLS_FRAME_NAME, CENTER_FRAME_NAME, mlsCenter);
        QString qCentreName = QString::fromStdString(CENTER_FRAME_NAME);
        window.displayGraph(graph, qCentreName);
        window.show();
        // Create the callback to compute collisions in those frames which have any collidable:
        //By now only done for the frames where we know there is a collidable
        //envire::core::FrameId colFrame = SMURF_COLLIDABLE_SPHERE_FRAME_NAME;
        envire::core::FrameId colFrame = SMURF_COLLIDABLE_BOX_FRAME_NAME;
        IterCollItem itCols, endCols;
        std::tie(itCols, endCols) = graph->getItems<CollisionItem>(colFrame);
        // TODO This you have to do for all the frames not only for one, but by now just pass the frame instead of the collidable to the EdgeCallBack method and move this retrieval of the collidables there
        if(itCols != endCols)
        {
            //Before: graph->subscribe(new EdgeCallBack(*graph, itCols->getData(), mlsItem->getData()));
            graph->subscribe(new EdgeCallBack(*graph, colFrame, mlsItem->getData()));
        }

        app.exec();
    }
    else
    {
        std::cout << "Only deserialization of sloped mls is supported";
    }
}
