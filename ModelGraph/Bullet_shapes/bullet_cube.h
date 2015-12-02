#ifndef BULLET_CUBE_H
#define BULLET_CUBE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_cube : public bullet_shape {
 public:
  // constructor
  explicit bullet_cube(const std::shared_ptr<ModelNode>& mnBox) {
    std::shared_ptr<BoxShape> pBox =
        std::dynamic_pointer_cast<BoxShape>(mnBox);
    double x_length = pBox->m_dBounds[0];
    double y_length = pBox->m_dBounds[1];
    double z_length = pBox->m_dBounds[2];
    double dMass = pBox->GetMass();
    double dRestitution = pBox->GetRestitution();
    Eigen::Matrix4d dPose;
    dPose = pBox->GetPoseMatrix();

    btVector3 bounds = btVector3(x_length*.5, y_length*.5, z_length*.5);
    bulletShape = std::make_shared<btBoxShape>(bounds);
    bulletMotionState = std::make_shared<NodeMotionState>(mnBox);
    bool isDynamic = (dMass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) {
      bulletShape->calculateLocalInertia(dMass, localInertia);
    }
    btRigidBody::btRigidBodyConstructionInfo  cInfo(
        dMass, bulletMotionState.get(),
        bulletShape.get(), localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    bulletBody->setRestitution(dRestitution);
    SetPose(dPose);
  }
};


#endif //  BULLET_CUBE_H
