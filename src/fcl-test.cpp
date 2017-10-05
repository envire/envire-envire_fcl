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

#include "Collision.hpp"

#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>

#include <envire_core/items/Item.hpp>
#include <envire_visualizer/EnvireVisualizerWindow.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Transform.hpp>

#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/QtThreadedWidget.hpp>
#include <plugin_manager/PluginLoader.hpp>

#include <Eigen/Geometry>
using namespace envire::viz;




struct EdgeCallBack : public envire::core::GraphEventDispatcher
{
    const envire::core::EnvireGraph &graph_;
    const MLSMapS& mls_;
public:
    EdgeCallBack(const envire::core::EnvireGraph& graph, const MLSMapS & mls) : graph_(graph), mls_(mls) {}
    void edgeModified(const envire::core::EdgeModifiedEvent& e) {
        std::cout << "Edge modified. Trafo=\n";
        std::cout << graph_.getEdgeProperty(e.edge).transform.getTransform().matrix();
        fcl::Transform3f trafo = graph_.getEdgeProperty(e.edge).transform.getTransform().cast<float>();
        fcl::CollisionRequestf request(10, true, 10, true);
        fcl::CollisionResultf result;
        //fcl::Spheref sphere(.25);
        fcl::Boxf box(1.0, 1.0, 1.0);
        fcl::collide_mls(mls_, trafo, &box, request, result);
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

    if(argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " mlsFileName\n";
        return 0;
    }
    plugin_manager::PluginLoader* loader = plugin_manager::PluginLoader::getInstance();
    envire::core::ItemBase::Ptr item;
    loader->createInstance("envire::core::Item<maps::grid::MLSMapSloped>", item);

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
    std::shared_ptr<envire::core::EnvireGraph> graph(new envire::core::EnvireGraph);
    graph->addFrame("A");
    graph->addFrame("B");
    graph->addItemToFrame("A", mlsItem);
    envire::core::Transform ab(base::Position(1, 1, 1), Eigen::Quaterniond::Identity());
    graph->addTransform("A", "B", ab);
    window.displayGraph(graph, "A");
    window.show();

    graph->subscribe(new EdgeCallBack(*graph, mlsItem->getData()));

    app.exec();
}
