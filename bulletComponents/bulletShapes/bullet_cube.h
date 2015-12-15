#pragma once

#include "bullet_shape.h"
#include <bullet/BulletCollision/CollisionShapes/btBoxShape.h>

class bullet_cube: public bullet_shape {

public:
  //constructor
  bullet_cube(double x_length, double y_length, double z_length, double dMass,
              double dRestitution, double* position, double* rotation)
  {
    btVector3 bounds = btVector3(x_length*.5, y_length*.5, z_length*.5);
    bulletShape = new btBoxShape(bounds);

    bulletMotionState = new btDefaultMotionState(btTransform::getIdentity());
    bool isDynamic = ( dMass != 0.f );
    btVector3 localInertia( 0, 0, 0 );
    if( isDynamic ){
      bulletShape->calculateLocalInertia( dMass, localInertia );
    }

    btRigidBody::btRigidBodyConstructionInfo  cInfo(dMass, bulletMotionState,
                                                    bulletShape, localInertia);
    bulletBody = new btRigidBody(cInfo);
    //    double dContactProcessingThreshold = 0.1;
    //    bulletBody->setContactProcessingThreshold( dContactProcessingThreshold );
    bulletBody->setRestitution( dRestitution );
    SetPose(position, rotation);
  }

  void getDrawData() {
    //  Front
    glColor3f(0,1,0);
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glTexCoord2f(0,0); glVertex3f(-1,-1,-1);
    glTexCoord2f(1,0); glVertex3f(-1,-1,+1);
    glTexCoord2f(1,1); glVertex3f(-1,+1,+1);
    glTexCoord2f(0,1); glVertex3f(-1,+1,-1);
    glEnd();
    //  Top
    glColor3f(0,1,1);
    glBegin(GL_QUADS);
    glNormal3f( 0,+1, 0);
    glTexCoord2f(0,0); glVertex3f(-1,+1,+1);
    glTexCoord2f(1,0); glVertex3f(+1,+1,+1);
    glTexCoord2f(1,1); glVertex3f(+1,+1,-1);
    glTexCoord2f(0,1); glVertex3f(-1,+1,-1);
    glEnd();
    //  Bottom
    glColor3f(1,0,1);
    glBegin(GL_QUADS);
    glNormal3f( 0,-1, 0);
    glTexCoord2f(0,0); glVertex3f(-1,-1,-1);
    glTexCoord2f(1,0); glVertex3f(+1,-1,-1);
    glTexCoord2f(1,1); glVertex3f(+1,-1,+1);
    glTexCoord2f(0,1); glVertex3f(-1,-1,+1);
    glEnd();
  }                                                         
};
