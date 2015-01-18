#ifndef BULLET_SPHERE_H
#define BULLET_SPHERE_H

#include <bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_sphere : public bullet_shape{

public:
  // constructor
  bullet_sphere(const std::shared_ptr<ModelNode>& mnSphere) {
    std::shared_ptr<SphereShape> pSphere =
        std::dynamic_pointer_cast<SphereShape>(mnSphere);
    double dRadius = pSphere->m_dRadius;
    double dMass = pSphere->GetMass();
    double dRestitution = pSphere->GetRestitution();
    Eigen::Matrix4d dPose;
    dPose = pSphere->GetPoseMatrix();

    bulletShape = std::make_shared<btSphereShape>(dRadius);
    bulletMotionState = std::make_shared<NodeMotionState>(mnSphere);
    bool isDynamic = (dMass != 0.f);
    btVector3 localInertia(0, 0, 0);
    if (isDynamic) {
        bulletShape->calculateLocalInertia(dMass, localInertia);
    }
    btRigidBody::btRigidBodyConstructionInfo  cInfo(
        dMass, bulletMotionState.get(),
        bulletShape.get(), localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold(dContactProcessingThreshold);
    bulletBody->setRestitution(dRestitution);
    SetPose(dPose);
  }

};


#endif //  BULLET_SPHERE_H
