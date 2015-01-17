#ifndef BULLET_SPHERE_H
#define BULLET_SPHERE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_sphere : public bullet_shape{

public:
  //constructor
  bullet_sphere(double dRadius, double dMass, double dRestitution,
                double *position, double* rotation){
    bulletShape = new btSphereShape(dRadius);

    bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
        bulletShape->calculateLocalInertia( dMass, localInertia );
    }

    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState,
                                                    bulletShape, localInertia);
    bulletBody = new btRigidBody(cInfo);
    double dContactProcessingThreshold = 0.001;
    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(position, rotation);
  }

};


#endif // BULLET_SPHERE_H
