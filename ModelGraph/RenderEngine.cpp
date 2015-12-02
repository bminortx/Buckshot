// Copyright (c) bminortx

#include <ModelGraph/RenderEngine.h>

void RenderEngine::Init(std::string sLocalSimName) {
  // Start our SceneGraph interface
  pangolin::CreateGlutWindowAndBind(sLocalSimName, 1024, 768);
  SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
  glClearColor(0, 0, 0, 1);
  glewInit();
}

///////////////////////

// Add to our list of SceneEntities
// This will be added later in 'AddToScene'
SceneGraph::GLObject* RenderEngine::AddNode(ModelNode *pNode) {
  //  Add our RaycastVehicle (a myriad of shapes)
  if (SimRaycastVehicle* pVehicle = dynamic_cast<SimRaycastVehicle*>(pNode)) {
    //  A RaycastVehicle is made of a cube and four cylinders.
    //  Since a vehicle is one shape in the PhysicsEngine, but five shapes
    //  in the RenderEngine, we must call PhysicsEngine::GetVehicleTransform
    //  anytime we want to update the rendering.

    //  Get the positions of every part of the car.
    std::vector<double> params = pVehicle->GetParameters();

    //  Were the meshes set for the car? If so, import those; else, use shapes.
    if (pVehicle->GetBodyMesh()!= "NONE" && pVehicle->GetWheelMesh()!= "NONE") {
      //  The chassis
      SceneGraph::GLMesh* chassis_mesh = new SceneGraph::GLMesh();
      chassis_mesh->Init(pVehicle->GetBodyMesh());
      chassis_mesh->SetPerceptable(true);
      Eigen::Vector3d vScale;
      Eigen::Vector3d vBodyDim = pVehicle->GetBodyMeshDim();
      vScale << (params[WheelBase]/vBodyDim(1)),
          1.5*params[Width]/vBodyDim(0),
          params[Height]/vBodyDim(2);
      chassis_mesh->SetScale(vScale);
      chassis_mesh->SetPose(pVehicle->GetPose());
      scene_entities_[pNode] = chassis_mesh;

      Eigen::Vector3d vWheelScale;
      Eigen::Vector3d vWheelDim = pVehicle->GetWheelMeshDim();
      vWheelScale << 2*params[WheelRadius]/vWheelDim(1),
          2*params[WheelRadius]/vWheelDim(2),
          params[WheelWidth]/vWheelDim(0);

      //  FL Wheel
      SceneGraph::GLMesh* FLWheel = new SceneGraph::GLMesh();
      FLWheel->Init(pVehicle->GetWheelMesh());
      FLWheel->SetScale(vWheelScale);
      FLWheel->SetPose(pVehicle->GetWheelPose(0));
      raycast_wheels_[pVehicle->GetName()+"@FLWheel"] = FLWheel;

      //  FR Wheel
      SceneGraph::GLMesh* FRWheel = new SceneGraph::GLMesh();
      FRWheel->Init(pVehicle->GetWheelMesh());
      FRWheel->SetPose(pVehicle->GetWheelPose(1));
      FRWheel->SetScale(vWheelScale);
      raycast_wheels_[pVehicle->GetName()+"@FRWheel"] = FRWheel;

      //  BL Wheel
      SceneGraph::GLMesh* BLWheel = new SceneGraph::GLMesh();
      BLWheel->Init(pVehicle->GetWheelMesh());
      BLWheel->SetPose(pVehicle->GetWheelPose(2));
      BLWheel->SetScale(vWheelScale);
      raycast_wheels_[pVehicle->GetName()+"@BLWheel"] = BLWheel;

      //  BR Wheel
      SceneGraph::GLMesh* BRWheel = new SceneGraph::GLMesh();
      BRWheel->Init(pVehicle->GetWheelMesh());
      BRWheel->SetPose(pVehicle->GetWheelPose(3));
      BRWheel->SetScale(vWheelScale);
      raycast_wheels_[pVehicle->GetName()+"@BRWheel"] = BRWheel;

      return chassis_mesh;
    } else {
      //  The chassis
      SceneGraph::GLBox* chassis = new SceneGraph::GLBox();
      chassis->SetExtent(params[WheelBase], params[Width], params[Height]);
      chassis->SetPose(pVehicle->GetPose());
      scene_entities_[pNode] = chassis;

      //  FL Wheel
      SceneGraph::GLCylinder* FLWheel = new SceneGraph::GLCylinder();
      FLWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      FLWheel->SetPose(pVehicle->GetWheelPose(0));
      raycast_wheels_[pVehicle->GetName()+"@FLWheel"] = FLWheel;

      //  FR Wheel
      SceneGraph::GLCylinder* FRWheel = new SceneGraph::GLCylinder();
      FRWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      FRWheel->SetPose(pVehicle->GetWheelPose(1));
      raycast_wheels_[pVehicle->GetName()+"@FRWheel"] = FRWheel;

      //  BL Wheel
      SceneGraph::GLCylinder* BLWheel = new SceneGraph::GLCylinder();
      BLWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      BLWheel->SetPose(pVehicle->GetWheelPose(2));
      raycast_wheels_[pVehicle->GetName()+"@BLWheel"] = BLWheel;

      //  BR Wheel
      SceneGraph::GLCylinder* BRWheel = new SceneGraph::GLCylinder();
      BRWheel->Init(params[WheelRadius], params[WheelRadius],
                    params[WheelWidth], 10, 10);
      BRWheel->SetPose(pVehicle->GetWheelPose(3));
      raycast_wheels_[pVehicle->GetName()+"@BRWheel"] = BRWheel;

      return chassis;
    }
  }

  //  Add our Shapes
  if (Shape* pShape = dynamic_cast<Shape*>(pNode)) {
    // Box
    if (BoxShape* pbShape = dynamic_cast<BoxShape*>(pShape)) {
      SceneGraph::GLBox* new_box = new SceneGraph::GLBox();
      new_box->SetExtent(pbShape->m_dBounds[0], pbShape->m_dBounds[1],
                         pbShape->m_dBounds[2]);
      new_box->SetPose(pbShape->GetPose());
      scene_entities_[pNode] = new_box;
      return new_box;
    } else if (CylinderShape* pbShape = dynamic_cast<CylinderShape*>(pShape)) {
      // Cylinder
      SceneGraph::GLCylinder* new_cylinder = new SceneGraph::GLCylinder();
      new_cylinder->Init(pbShape->m_dRadius, pbShape->m_dRadius,
                         pbShape->m_dHeight, 32, 1);
      new_cylinder->SetPose(pbShape->GetPose());
      scene_entities_[pNode] = new_cylinder;
      return new_cylinder;
    } else if (PlaneShape* pbShape = dynamic_cast<PlaneShape*>(pShape)) {
      // Plane
      SceneGraph::GLGrid* new_plane = new SceneGraph::GLGrid();
      new_plane->SetNumLines(20);
      new_plane->SetLineSpacing(1);
      Eigen::Vector3d eig_norm;
      eig_norm << pbShape->m_dNormal[0],
          pbShape->m_dNormal[1],
          pbShape->m_dNormal[2];
      new_plane->SetPlane(eig_norm);
      scene_entities_[pNode] = new_plane;
      return new_plane;
    } else if (MeshShape* pbShape = dynamic_cast<MeshShape*>(pShape)) {
      // Mesh
      SceneGraph::GLMesh* new_mesh = new SceneGraph::GLMesh();
      new_mesh->Init(pbShape->GetFileDir());
      new_mesh->SetPerceptable(true);
      new_mesh->SetScale(pbShape->GetScale());
      new_mesh->SetPose(pbShape->GetPose());
      scene_entities_[pNode] = new_mesh;
      return new_mesh;
    } else if (HeightmapShape* pbShape =
               dynamic_cast<HeightmapShape*>(pShape)) {
      // Heightmap
      SceneGraph::GLHeightmap* new_map =
          new SceneGraph::GLHeightmap(pbShape->x_data_, pbShape->y_data_,
                                      pbShape->z_data_, pbShape->row_count_,
                                      pbShape->col_count_);
      scene_entities_[pNode] = new_map;
      return new_map;
    } else if (LightShape* pbShape = dynamic_cast<LightShape*>(pShape)) {
      // COLLISION-FREE OBJECTS
      // Light
      SceneGraph::GLLight* new_light =
          new SceneGraph::GLLight(pbShape->GetPose()(0, 0),
                                  pbShape->GetPose()(1, 0),
                                  pbShape->GetPose()(2, 0));
      scene_entities_[pNode] = new_light;
      return new_light;
    } else if (WaypointShape* pbShape = dynamic_cast<WaypointShape*>(pShape)) {
      // Waypoint
      SceneGraph::GLWayPoint* new_waypoint =
          new SceneGraph::GLWayPoint();
      new_waypoint->SetPose(pbShape->GetPose());
      new_waypoint->SetVelocity(pbShape->GetVelocity());
      new_waypoint->SetScale(pbShape->GetVelocity());
      scene_entities_[pNode] = new_waypoint;
      return new_waypoint;
    }
  }
}

///////////////////////////////////////

void RenderEngine::AddDevices(const SimDevices& Devices) {
  for (std::map<std::string, SimDeviceInfo*>::const_iterator it =
           Devices.sim_device_map_.begin();
       it != Devices.sim_device_map_.end();
       it++) {
    SimDeviceInfo* Device = it->second;
    if (Device->m_sDeviceType == "Camera") {
      SimCamera* pSimCam = static_cast<SimCamera*>(Device);
      //  Initialize the cameras with SceneGraph
      pSimCam->init(&gl_graph_);
      //  Match devices with their ModelNodes
      for (std::map<ModelNode*, SceneGraph::GLObject*>::iterator jj =
               scene_entities_.begin();
           jj != scene_entities_.end();
           jj++) {
        ModelNode* pNode = jj->first;
        if (isCameraBody(pNode->GetName(), pSimCam->GetBodyName())) {
          scene_cameras_[pSimCam] = pNode;
        }
      }
    }
  }
}

/////////////////////////////////////////////////

bool RenderEngine::UpdateCameras() {
  bool bStatus = false;
  for (std::map<SimCamera*, ModelNode*>::iterator it = scene_cameras_.begin();
       it != scene_cameras_.end();
       it++) {
    SimCamera* Device = it->first;
    Device->Update();
    bStatus = true;
  }
  return bStatus;
}


/////////////////////////////////////////////////

// Scan all SimDevices and send the simulated camera images to Pangolin.
// Right now, we can only support up to two windows.

// TODO(anyone): Fairly certain that there's a glitch here.
// Go through and correct the Image buffers for content.
void RenderEngine::SetImagesToWindow() {
  int WndCounter = 0;
  for (std::map<SimCamera*, ModelNode*>::iterator it = scene_cameras_.begin();
       it != scene_cameras_.end();
       it++) {
    SimCamera* Device = it->first;
    if (Device->m_bDeviceOn == true) {
      SimCamera* pSimCam = dynamic_cast<SimCamera*>(Device);
      SceneGraph::ImageView* ImageWnd;
      //  get pointer to window
      (WndCounter == 0) ? ImageWnd = left_cam_image_ :
          ImageWnd = right_cam_image_;
      WndCounter++;
      //  Set image to window
      //  DEPTH
      if (pSimCam->glcamera_type_ == SceneGraph::eSimCamDepth) {
        float* pImgbuf =
            static_cast<float*>(malloc(pSimCam->image_width_ *
                                       pSimCam->image_height_ *
                                       sizeof(float)));
        bool success = pSimCam->capture(pImgbuf);
        if (success) {
          ImageWnd->SetImage(pImgbuf, pSimCam->image_width_,
                             pSimCam->image_height_,
                             GL_INTENSITY, GL_LUMINANCE, GL_FLOAT);
          free(pImgbuf);
        }
      } else if (pSimCam->glcamera_type_ == SceneGraph::eSimCamRGB) {
        //  RGB
        char* pImgbuf =
            static_cast<char*>(malloc(pSimCam->image_width_ *
                                      pSimCam->image_height_ * 3));
        if (pSimCam->capture(pImgbuf) == true) {
          ImageWnd->SetImage(pImgbuf, pSimCam->image_width_,
                             pSimCam->image_height_,
                             GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE);
          free(pImgbuf);
        }
      } else if (pSimCam->glcamera_type_ == SceneGraph::eSimCamLuminance) {
        //  GREY
        char* pImgbuf =
            static_cast<char*>(malloc(pSimCam->image_width_ *
                                      pSimCam->image_height_));
        if (pSimCam->capture(pImgbuf) == true) {
          ImageWnd->SetImage(pImgbuf, pSimCam->image_width_,
                             pSimCam->image_height_,
                             GL_INTENSITY, GL_LUMINANCE, GL_UNSIGNED_BYTE);
          free(pImgbuf);
        }
      }
    }
  }
}

///////////////////////////////////////

void RenderEngine::AddToScene() {
  //  Add shapes
  std::map<ModelNode*, SceneGraph::GLObject* >::iterator it;
  for (it = scene_entities_.begin(); it != scene_entities_.end(); it++) {
    SceneGraph::GLObject* p = it->second;
    gl_graph_.AddChild(p);
  }
  //  If we have them, add wheels
  std::map<std::string, SceneGraph::GLObject* >::iterator jj;
  for (jj = raycast_wheels_.begin(); jj != raycast_wheels_.end(); jj++) {
    SceneGraph::GLObject* w = jj->second;
    gl_graph_.AddChild(w);
  }
}

// This method helps add objects even during simulation
void RenderEngine::AddNewShape(SceneGraph::GLObject* object) {
  gl_graph_.AddChild(object);
}

///////////////////////////////////////

void RenderEngine::CompleteScene(bool bEnableCameraView = false) {
  const SceneGraph::AxisAlignedBoundingBox bbox =
      gl_graph_.ObjectAndChildrenBounds();
  const Eigen::Vector3d center = bbox.Center();
  const double size = bbox.Size().norm();
  const double far = 15*size;
  const double near = far / 1E3;

  //  Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState stacks(
      pangolin::ProjectionMatrix(1024, 768, 420, 420,
                                 1024/2, 768/2, 0.1, 1000),
      //  pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, near, far),
      pangolin::ModelViewLookAt(center(0), center(1) + 7,
                                center(2) + 6,
                                center(0), center(1), center(2),
                                pangolin::AxisZ));
  gl_stacks_ = stacks;

  //  We define a new view which will reside within the container.

  //  We set the views location on screen and add a handler which will
  //  let user input update the model_view matrix (stacks3d) and feed through
  //  to our scenegraph
  gl_view_ = new pangolin::View(0.0);
  gl_view_->SetBounds(0.0, 1.0, 0.0, 1.0/*, -640.0f/480.0f*/);
  gl_view_->SetHandler(new SceneGraph::HandlerSceneGraph(
      gl_graph_, gl_stacks_));
  gl_view_->SetDrawFunction(SceneGraph::ActivateDrawFunctor(
      gl_graph_, gl_stacks_));

  //  Add our views as children to the base container.
  pangolin::DisplayBase().AddDisplay(*gl_view_);

  if (bEnableCameraView == true) {
    is_camera_view_ = true;
    /// window for display image capture from SimCamera
    left_cam_image_ = new SceneGraph::ImageView(true, true);
    left_cam_image_->SetBounds(0.0, 0.5, 0.75, 1.0/*, 512.0f/384.0f*/);

    /// window for display image capture from SimCamera
    right_cam_image_ = new SceneGraph::ImageView(true, true);
    right_cam_image_->SetBounds(0.5, 1.0, 0.75, 1.0/*, 512.0f/384.0f */);

    pangolin::DisplayBase().AddDisplay(*left_cam_image_);
    pangolin::DisplayBase().AddDisplay(*right_cam_image_);
  } else {
    is_camera_view_ = false;
  }
}

////////////////////////////////////////////////////

bool RenderEngine::isCameraBody(std::string BodyName, std::string CameraName) {
  bool inthere = false;
  std::size_t found = BodyName.find(CameraName);
  if (found!= std::string::npos) {
    inthere = true;
  }
  return inthere;
}

////////////////////////////////////////////////////

void RenderEngine::UpdateScene() {
  // React to changes in the PhysicsEngine
  std::map<ModelNode*, SceneGraph::GLObject*>::iterator it;
  for (it = scene_entities_.begin(); it != scene_entities_.end(); it++) {
    ModelNode* mn = it->first;
    SceneGraph::GLObject* p = it->second;
    p->SetPose(mn->GetPose());
    //  Update all of our tires.
    if (SimRaycastVehicle* pVehicle = dynamic_cast<SimRaycastVehicle*>(mn)) {
      std::map<std::string, SceneGraph::GLObject*>::iterator jj;
      for (jj = raycast_wheels_.begin(); jj != raycast_wheels_.end(); jj++) {
        std::string name = jj->first;
        SceneGraph::GLObject* wheel = jj->second;
        if (name == pVehicle->GetName()+"@FLWheel") {
          wheel->SetPose(pVehicle->GetWheelPose(0));
        } else if (name == pVehicle->GetName()+"@FRWheel") {
          wheel->SetPose(pVehicle->GetWheelPose(1));
        } else if (name == pVehicle->GetName()+"@BLWheel") {
          wheel->SetPose(pVehicle->GetWheelPose(2));
        } else if (name == pVehicle->GetName()+"@BRWheel") {
          wheel->SetPose(pVehicle->GetWheelPose(3));
        }
      }
    }
  }
  // Change the views in the Cameras
  std::map<SimCamera*, ModelNode*>::iterator jj;
  for (jj = scene_cameras_.begin(); jj != scene_cameras_.end(); jj++) {
    SimCamera* pCamera = jj->first;
    ModelNode* pNode = jj->second;
    pCamera->m_vPose = pNode->GetPose();
  }
  if (UpdateCameras() == true && is_camera_view_ == true) {
    SetImagesToWindow();
  }
}
