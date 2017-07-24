/*
 * fcl-collidable-test-robot.cpp
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
#define ENV_AUTOPROJ_ROOT "AUTOPROJ_CURRENT_ROOT"
#define ASGUARD_PATH std::string("/models/robots/asguard_v4/smurf/asguard_v4.smurf")

using namespace envire::viz;
using CollisionType = smurf::Collidable;
using CollisionItem = envire::core::Item<CollisionType>;
using IterCollItem = envire::core::EnvireGraph::ItemIterator<CollisionItem>;

struct EdgeCallBack : public envire::core::GraphEventDispatcher
{
    const envire::core::EnvireGraph &graph_;
    const std::vector<envire::core::FrameId> &colFrames_;
    const MLSMapS& mls_;

public:
    EdgeCallBack(const envire::core::EnvireGraph& graph, std::vector<envire::core::FrameId> &colFrames, const MLSMapS & mls) : graph_(graph), colFrames_(colFrames), mls_(mls) {
    }
    void edgeModified(const envire::core::EdgeModifiedEvent& e) {
        /*
         * Each time a edge is modified, this method iterates over the frames
         * which have a collision object and computes whether collision with
         * the MLS takes place or not.
         */
        smurf::Collidable collidable;
        IterCollItem itCols, endCols;
        for(unsigned int frameIndex = 0; frameIndex<colFrames_.size(); ++frameIndex)
        {
            itCols = graph_.getItem<CollisionItem>(colFrames_[frameIndex]); 
            std::cout << "Collision related to frame " << colFrames_[frameIndex] << std::endl;
            collidable = itCols->getData();
            // Transformation must be from the mls frame to the colision object frame
            envire::core::Transform tfColCen = graph_.getTransform(MLS_FRAME_NAME, colFrames_[frameIndex]);
            fcl::Transform3f trafo = tfColCen.transform.getTransform().cast<float>();
            std::cout << "Edge modified. Trafo=\n" << std::endl;
            std::cout << tfColCen.transform.getTransform().matrix() << std::endl;
            fcl::CollisionRequestf request(10, true, 10, true);
            fcl::CollisionResultf result;
            urdf::Collision collision = collidable.getCollision();
            bool collisionComputed = true;
            switch (collision.geometry->type){
                case urdf::Geometry::SPHERE:
                    {
                        std::cout << "Collision with a sphere" << std::endl;
                        boost::shared_ptr<urdf::Sphere> sphereUrdf = boost::dynamic_pointer_cast<urdf::Sphere>(collision.geometry);
                        fcl::Spheref sphere(sphereUrdf->radius);
                        fcl::collide_mls(mls_, trafo, &sphere, request, result);
                        break;
                    }
                case urdf::Geometry::BOX:
                    {
                        std::cout << "Collision with a box" << std::endl;
                        boost::shared_ptr<urdf::Box> boxUrdf = boost::dynamic_pointer_cast<urdf::Box>(collision.geometry);
                        fcl::Boxf box(boxUrdf->dim.x, boxUrdf->dim.y, boxUrdf->dim.z);
                        fcl::collide_mls(mls_, trafo, &box, request, result);
                        break;
                    }
                default:
                    std::cout << "Collision with the selected geometry type not implemented" << std::endl;
                    collisionComputed = false;
            }
            if (collisionComputed)
            {
                std::cout << "\nisCollision()==" << result.isCollision();
                for(size_t i=0; i< result.numContacts(); ++i)
                {
                    const auto & cont = result.getContact(i);
                    std::cout << "\n" << cont.pos.transpose() << "; " << cont.normal.transpose() << "; " << cont.penetration_depth;
                }
            std::cout << std::endl;
            }
        }
    }
};

int main(int argc, char **argv) {

    // Use smurf_loader to load an smurf file
    const std::string path = std::getenv(ENV_AUTOPROJ_ROOT) + ASGUARD_PATH;
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
    viz.write(*(graphLoader.getGraph()), "fcl-collidable-test-robot.dot");
    
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
        graph->addFrame(MLS_FRAME_NAME);
        graph->addItemToFrame(MLS_FRAME_NAME, mlsItem);
        envire::core::Transform mlsCenter(base::Position(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
        graph->addTransform(MLS_FRAME_NAME, CENTER_FRAME_NAME, mlsCenter);
        QString qCentreName = QString::fromStdString(CENTER_FRAME_NAME);
        window.displayGraph(graph, qCentreName);
        window.show();
        // Find out the frames which contain a collidablem put them in a vector and pass it to the callback
        std::vector<envire::core::FrameId> colFrames;
        envire::core::EnvireGraph::vertex_iterator  it, end;
        std::tie(it, end) = graph->getVertices();
        for(; it != end; ++it)
        {
            // See if the vertex has collision objects
            IterCollItem itCols, endCols;
            std::tie(itCols, endCols) = graph->getItems<CollisionItem>(*it);
            if(itCols != endCols)
            {
                envire::core::FrameId colFrame = graph->getFrameId(*it);
                colFrames.push_back(colFrame);
                std::cout << "Collision items found in frame " << colFrame << std::endl;
            }
        }
        EdgeCallBack * callBack = new EdgeCallBack(*graph, colFrames, mlsItem->getData());
        graph->subscribe(callBack);
        app.exec();
    }
    else
    {
        std::cout << "Only deserialization of sloped mls is supported";
    }
}
