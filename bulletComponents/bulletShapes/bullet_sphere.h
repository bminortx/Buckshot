#pragma once

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btSphereShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "bullet_shape.h"

class bullet_sphere : public bullet_shape{

public:
  //constructor
  bullet_sphere(double dRadius, double dMass, double dRestitution,
                double *position, double* rotation){
    _radius = dRadius;
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

  /// OpenGL stuff
  void getDrawData() {
    glColor3f( 1, 1, 1);
    GLUquadricObj *quadric;
    quadric = gluNewQuadric();
    gluQuadricDrawStyle(quadric, GLU_FILL);
    gluSphere(quadric, _radius, 36, 18);
    gluDeleteQuadric(quadric); 
  }

  double _radius;
  
};
