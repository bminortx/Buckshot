#ifndef BULLET_CYLINDER_H
#define BULLET_CYLINDER_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btCylinderShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_cylinder : public bullet_shape{

public:
  //constructor
  bullet_cylinder(const std::shared_ptr<ModelNode>& mnCylinder){
    std::shared_ptr<CylinderShape> pCylinder =
        std::dynamic_pointer_cast<CylinderShape>(mnCylinder);
    double dRadius = pCylinder->m_dRadius;
    double dHeight = pCylinder->m_dHeight;
    double dMass = pCylinder->GetMass();
    double dRestitution = pCylinder->GetRestitution();
    Eigen::Matrix4d dPose;
    dPose = pCylinder->GetPoseMatrix();

    bulletShape = std::make_shared<btCylinderShapeZ>(
        btVector3(dRadius, dRadius, dHeight/2));
    bulletMotionState = std::make_shared<NodeMotionState>(mnCylinder);
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
        bulletShape->calculateLocalInertia( dMass, localInertia );
    }
    btRigidBody::btRigidBodyConstructionInfo cInfo(
        dMass, bulletMotionState.get(),
        bulletShape.get(), localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(dPose);
  }

};

#endif // BULLET_CYLINDER_H
