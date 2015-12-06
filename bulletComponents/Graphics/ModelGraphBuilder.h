#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <Graphics/RenderEngine.h>
#include <Graphics/PhysicsEngine.h>

class ModelGraphBuilder{
public:

    /////////////////////////////////////////
    /// INITIALIZER FOR THE WORLD
    /////////////////////////////////////////

    /// PHYSICS_ENGINE CONSTRUCTORS
    void AssociatePhysics();
    /// RENDER_ENGINE CONSTRUCTORS
    void RenderGraph();
    void Init(const std::vector<std::shared_ptr<ModelNode> >& models,
	      const bool camera_option,
	      const bool render_option);

    /// UPDATE THE SCENE
    void UpdateScene();

    /// MEMBER VARIABLES
    std::vector<std::shared_ptr<ModelNode> >& models_;
    bool render_option_;
    bool camera_option_;
    std::shared_ptr<PhysicsEngine> physics_engine_;
    std::shared_ptr<RenderEngine> render_engine_;
};

#endif // MODELGRAPHBUILDER_H
