#ifndef BULLET_PLANE_H
#define BULLET_PLANE_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <ModelGraph/Bullet_shapes/bullet_shape.h>

//Constructs a Bullet btStaticPlaneShape.

class bullet_plane: public bullet_shape{

public:
  //constructor
  explicit bullet_plane(const std::shared_ptr<ModelNode>& mnPlane){
    std::shared_ptr<PlaneShape> pPlane =
        std::dynamic_pointer_cast<PlaneShape>(mnPlane);
    std::vector<double> dNormal = pPlane->m_dNormal;

    //Just make a flat plain
    bulletShape = std::make_shared<btStaticPlaneShape>(
          btVector3(dNormal[0], dNormal[1], dNormal[2]), 0);
    bulletMotionState = std::make_shared<NodeMotionState>(mnPlane);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0, bulletMotionState.get(),
                                                   bulletShape.get(),
                                                   btVector3(0, 0, 0));
    bulletBody = std::make_shared<btRigidBody>(cInfo);
  }

};

#endif // BULLET_PLANE_H
