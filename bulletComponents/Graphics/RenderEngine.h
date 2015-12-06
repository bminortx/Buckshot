// Copyright (c) bminortx

#ifndef SIMBA_MODELGRAPH_RENDERENGINE_H_
#define SIMBA_MODELGRAPH_RENDERENGINE_H_

// TODO: OpenGL Graphics here

// All of our bullet objects
#include <bulletStructs/Shape.h>
#include <bulletStructs/Constraint.h>
#include <bulletStructs/SimRaycastVehicle.h>
#include <SimDevices/SimDevices.h>

#include <map>
#include <string>

class RenderEngine {
 public:
  void Init(std::string sLocalSimName);

  // Add to our list of SceneEntities
  SceneGraph::GLObject* AddNode(ModelNode *pNode);

  // Add sensors and cameras to the Scene
  void AddDevices(const SimDevices& Devices);
  bool UpdateCameras();
  void SetImagesToWindow();

  // Pass all SceneEntities (and RaycastWheels) at one time into
  // the Scene
  void AddToScene();
  void AddNewShape(SceneGraph::GLObject* object);

  // Complete the SceneGraph and Pangolin initialization
  void CompleteScene(bool bEnableCameraView);

  bool isCameraBody(std::string BodyName, std::string CameraName);

  // Update the Scene by one timestep
  void UpdateScene();


  /// MEMBER VARIABLES
  std::map<ModelNode*, SceneGraph::GLObject*> scene_entities_;
  std::map<std::string, SceneGraph::GLObject*> raycast_wheels_;
  std::map<SimCamera*, ModelNode*> scene_cameras_;
  SceneGraph::GLSceneGraph gl_graph_;
  SceneGraph::ImageView* left_cam_image_;
  SceneGraph::ImageView* right_cam_image_;
  pangolin::View* gl_view_;
  pangolin::OpenGlRenderState gl_stacks_;
  bool is_camera_view_;
};

#endif  // SIMBA_MODELGRAPH_RENDERENGINE_H_
