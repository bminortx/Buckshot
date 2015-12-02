#include <ModelGraph/ModelGraphBuilder.h>

/////////////////////////////////////////
/// PHYSICS FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::AssociateWorldPhysics(const SimWorld& m_SimWorld) {
  for (unsigned int ii = 0; ii < m_SimWorld.models_.size(); ii++) {
    std::shared_ptr<ModelNode> part = m_SimWorld.models_.at(ii);
    if (dynamic_cast<Shape*>(part.get())) {
      physics_engine_->RegisterObject(part);
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsShapes(const SimRobot& sim_robot) {
  for (unsigned int ii = 0; ii < sim_robot.GetParts().size(); ii++) {
    std::shared_ptr<ModelNode> part = sim_robot.GetParts().at(ii);
    if (dynamic_cast<Shape*>(part.get())) {
      Eigen::Vector6d newPose = robot_pose_ + part->GetPose();
      part->SetPose( newPose );
      physics_engine_->RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociatePhysicsConstraints(
    const SimRobot& sim_robot) {
  for (unsigned int ii = 0; ii < sim_robot.GetParts().size(); ii++) {
    std::shared_ptr<ModelNode> part = sim_robot.GetParts().at(ii);
    // Constraints are in their own reference frame, so they
    // don't need modification.
    if (dynamic_cast<Constraint*>(part.get())) {
      physics_engine_->RegisterObject( part );
    }
  }
}

void ModelGraphBuilder::AssociateRobotPhysics(const SimRobot& sim_robot) {
  AssociatePhysicsShapes(sim_robot);
  AssociatePhysicsConstraints(sim_robot);
}


void ModelGraphBuilder::AssociateDevices(const SimDevices& m_SimDevices) {
  for (std::map<std::string, SimDeviceInfo*>::const_iterator it =
      m_SimDevices.sim_device_map_.begin();
      it != m_SimDevices.sim_device_map_.end();
      it++) {
    SimDeviceInfo* pDevice = it->second;
    physics_engine_->RegisterDevice(pDevice);
  }
}

/////////////////////////////////////////
/// RENDER FUNCTIONS
/////////////////////////////////////////

void ModelGraphBuilder::RenderWorldGraph(const SimWorld m_SimWorld) {
  for (unsigned int ii = 0; ii < m_SimWorld.models_.size(); ii++) {
    std::shared_ptr<ModelNode> part = m_SimWorld.models_.at(ii);
    render_engine_->AddNode(part.get());
  }
}

void ModelGraphBuilder::RenderRobotGraph(const SimRobot sim_robot) {
  for (unsigned int ii = 0; ii < sim_robot.GetParts().size(); ii++) {
    std::shared_ptr<ModelNode> part = sim_robot.GetParts().at(ii);
    render_engine_->AddNode(part.get());
    Eigen::Vector6d ChildWorldPose;
    for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
      ChildWorldPose = _T2Cart(
          part->GetPoseMatrix() * part->model_children_[ii]->GetPoseMatrix());
      part->SetPose(ChildWorldPose);
    }
  }
}

/////////////////////////////////////////
/// UPDATE FUNCTIONS
/// This just adds to the RenderGraph right now.
/// I'll worry about the Physics later
/////////////////////////////////////////

void ModelGraphBuilder::CheckForNewShapes() {
  std::mutex modelgraph_mutex;
  std::lock_guard<std::mutex> lock(modelgraph_mutex);
  int new_parts = sim_robot_.GetNewPartsBit();
  if (new_parts > 0) {
    for (unsigned int ii = new_parts;
         ii < sim_robot_.GetParts().size(); ii++) {
      std::shared_ptr<ModelNode> part = sim_robot_.GetParts().at(ii);
      SceneGraph::GLObject* object = render_engine_->AddNode(part.get());
      render_engine_->AddNewShape(object);
      Eigen::Vector6d ChildWorldPose;
      for (unsigned int ii = 0; ii < part->NumChildren(); ii++ ) {
        ChildWorldPose = _T2Cart(
            part->GetPoseMatrix() * part->model_children_[ii]->GetPoseMatrix());
        part->SetPose(ChildWorldPose);
      }
    }
    sim_robot_.SetNewPartsBit(0);
  }
}

////////////////////////////////////////

void ModelGraphBuilder::Init(const SimWorld& world_model,
                             const SimRobot& sim_robot,
                             const SimDevices& m_SimDevices,
                             const std::string sSimName,
                             const bool debug, const bool render,
                             const bool bEnableCameraView=false) {
  debug_status_ = debug;
  render_status_ = render;
  // world_model_ = world_model;
  sim_robot_ = sim_robot;
  physics_engine_ = std::make_shared<PhysicsEngine>();
  physics_engine_->Init();
  if (render_status_) {
    render_engine_ = std::make_shared<RenderEngine>();
    render_engine_->Init(sSimName);
  }
  if (sim_robot.GetStateKeeperStatus()) {
    // Get the PoseRW from the StateKeeper, and not the World.XML file.
  } else {
    robot_pose_ << world_model.robot_pose_[0], world_model.robot_pose_[1],
        world_model.robot_pose_[2], world_model.robot_pose_[3],
        world_model.robot_pose_[4], world_model.robot_pose_[5];
  }
  // We move the parts around to their respective places in
  // Associate****Physics; RenderWorldGraph should keep the same places.
  // This inheritance pattern is what makes the ModelNode class so important.
  AssociateWorldPhysics(world_model);
  AssociateRobotPhysics(sim_robot);
  AssociateDevices(m_SimDevices);
  if (render_status_) {
    RenderWorldGraph(world_model);
    RenderRobotGraph(sim_robot);
    render_engine_->AddToScene();
    render_engine_->CompleteScene(bEnableCameraView);
    render_engine_->AddDevices(m_SimDevices);
  }
}

//////////

void ModelGraphBuilder::UpdateScene() {
  CheckForNewShapes();
  if (debug_status_) {
    physics_engine_->DebugDrawWorld();
  } else {
    physics_engine_->StepSimulation();
  }
  if (render_status_) {
    render_engine_->UpdateScene();
  }
}
