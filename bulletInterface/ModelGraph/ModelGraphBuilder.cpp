#include <ModelGraph/ModelGraphBuilder.h>

/////////////////////////////////////////
/// PHYSICS FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::AssociatePhysics() {
  for (unsigned int ii = 0; ii < models_.size(); ii++) {
    boost::shared_ptr<ModelNode> part = models_.at(ii);
    if (dynamic_cast<Shape*>(part.get())) {
      Eigen::Vector6d newPose = robot_pose_ + part->GetPose();
      part->SetPose( newPose );
      physics_engine_->RegisterObject( part );
    }
    if (dynamic_cast<Constraint*>(part.get())) {
      physics_engine_->RegisterObject( part );
    }
  }
}

/////////////////////////////////////////
/// RENDER FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::RenderGraph() {
  for (unsigned int ii = 0; ii < models_.size(); ii++) {
    boost::shared_ptr<ModelNode> part = models_.at(ii);
    render_engine_->AddNode(part.get());
    Eigen::Vector6d ChildWorldPose;
    for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
      ChildWorldPose = _T2Cart(
          part->GetPoseMatrix() * part->model_children_[ii]->GetPoseMatrix());
      part->SetPose(ChildWorldPose);
    }
  }
}

////////////////////////////////////////

void ModelGraphBuilder::Init(
    const std::vector<boost::shared_ptr<ModelNode> >& models,
    const bool camera_option,
    const bool render_option) :
    render_option_(render_option), models_(models),
    camera_option_(camera_option){
  physics_engine_ = boost::make_shared<PhysicsEngine>();
  physics_engine_->Init();
  if (render_status_) {
    render_engine_ = boost::make_shared<RenderEngine>();
    render_engine_->Init(sSimName);
  }
  AssociatePhysics();
  if (render_option_) {
    RenderGraph();
    render_engine_->AddToScene();
    render_engine_->CompleteScene(camera_option_);
    // render_engine_->AddDevices(m_SimDevices);
  }
}

//////////

void ModelGraphBuilder::UpdateScene() {
  physics_engine_->StepSimulation();
  if (render_status_) {
    render_engine_->UpdateScene();
  }
}
