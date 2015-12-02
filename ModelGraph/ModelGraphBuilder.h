#ifndef MODELGRAPHBUILDER_H
#define MODELGRAPHBUILDER_H

#include <mutex>

#include <ModelGraph/RenderEngine.h>
#include <ModelGraph/PhysicsEngine.h>
#include <SimRobots/SimRobot.h>
#include <SimRobots/SimWorld.h>
#include <SimDevices/SimDevices.h>

class ModelGraphBuilder{
 public:

  /////////////////////////////////////////
  /// INITIALIZER FOR THE WORLD
  /////////////////////////////////////////

  void Init(const SimWorld& m_WorldModel,
            const SimRobot& m_SimRobot,
            const SimDevices& m_SimDevices,
            const std::string sSimName,
            const bool debug, const bool render,
            const bool bEnableCameraView);

  /// PHYSICS_ENGINE CONSTRUCTORS
  void AssociateWorldPhysics(const SimWorld& m_SimWorld);
  void AssociatePhysicsShapes(const SimRobot& m_SimRobot);
  void AssociatePhysicsConstraints(const SimRobot& m_SimRobot);
  void AssociateRobotPhysics(const SimRobot& m_SimRobot);
  void AssociateDevices(const SimDevices& m_SimDevices);

  /// RENDER_ENGINE CONSTRUCTORS
  void RenderWorldGraph(const SimWorld m_SimWorld);
  void RenderRobotGraph(const SimRobot m_SimRobot);

  /// UPDATE THE SCENE
  void CheckForNewShapes();
  void UpdateScene();

  /// MEMBER VARIABLES
  Eigen::Vector6d robot_pose_;
  std::shared_ptr<PhysicsEngine> physics_engine_;
  std::shared_ptr<RenderEngine> render_engine_;
  // SimWorld& world_model_;
  SimRobot sim_robot_;
  bool debug_status_;
  bool render_status_;

};

#endif // MODELGRAPHBUILDER_H
