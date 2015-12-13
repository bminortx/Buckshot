#ifndef BULLET_CYLINDER_H
#define BULLET_CYLINDER_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btCylinderShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_cylinder : public bullet_shape{

public:
  //constructor
  bullet_cylinder(double dRadius, double dHeight, double dMass, double dRestitution,
                double *position, double* rotation){
    bulletShape = std::make_shared<btCylinderShapeZ>(btVector3(dRadius, dRadius, dHeight/2));
    bulletMotionState = std::make_shared<btDefaultMotionState>(btTransform::getIdentity());
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
        bulletShape->calculateLocalInertia( dMass, localInertia );
    }

    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState.get(),
                                                    bulletShape.get(), localInertia);
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(position, rotation);
  }

  void set_vertex_data() {}

};

#endif // BULLET_CYLINDER_H
