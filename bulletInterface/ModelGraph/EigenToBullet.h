// Copyright (c) bminortx

#ifndef SIMBA_MODELGRAPH_EIGENTOBULLET_H_
#define SIMBA_MODELGRAPH_EIGENTOBULLET_H_

#include <bullet/btBulletDynamicsCommon.h>
#include <Eigen/Eigen>
#include <bulletStructs/EigenHelpers.h>

//////////////////////////////////////////////////////////
///
/// EIGEN-TO-BULLET-TO-EIGEN CONVERTERS
///
//////////////////////////////////////////////////////////

inline Eigen::Matrix4d
getInverseTransformation(const Eigen::Matrix4d &transformation) {
  Eigen::Matrix4d transformation_inverse;
  float tx = transformation(0, 3);
  float ty = transformation(1, 3);
  float tz = transformation(2, 3);

  transformation_inverse(0, 0) = transformation(0, 0);
  transformation_inverse(0, 1) = transformation(1, 0);
  transformation_inverse(0, 2) = transformation(2, 0);
  transformation_inverse(0, 3) = -(transformation(0, 0) * tx
                                     + transformation(0, 1) * ty
                                     + transformation(0, 2) * tz);

  transformation_inverse(1, 0) = transformation(0, 1);
  transformation_inverse(1, 1) = transformation(1, 1);
  transformation_inverse(1, 2) = transformation(2, 1);
  transformation_inverse(1, 3) = -(transformation(1, 0) * tx
                                     + transformation(1, 1) * ty
                                     + transformation(1, 2) * tz);

  transformation_inverse(2, 0) = transformation(0, 2);
  transformation_inverse(2, 1) = transformation(1, 2);
  transformation_inverse(2, 2) = transformation(2, 2);
  transformation_inverse(2, 3) = -(transformation(2, 0) * tx
                                     + transformation(2, 1) * ty
                                     + transformation(2, 2) * tz);

  transformation_inverse(3, 0) = 0;
  transformation_inverse(3, 1) = 0;
  transformation_inverse(3, 2) = 0;
  transformation_inverse(3, 3) = 1;
  return transformation_inverse;
}

inline Eigen::Matrix<double, 4, 4> toEigen(const btTransform& T) {
  Eigen::Matrix<btScalar, 4, 4> eT;
  T.getOpenGLMatrix(eT.data());
  return eT.cast<double>();
}

inline btTransform toBullet(const Eigen::Matrix<double, 4, 4>& T) {
  btTransform bT;
  Eigen::Matrix<btScalar, 4, 4> eT = T.cast<btScalar>();
  bT.setFromOpenGLMatrix(eT.data());
  return bT;
}

inline btVector3 toBulletVec3(const Eigen::Vector3d& v) {
  btVector3 bv;
  bv.setX(v(0));
  bv.setY(v(1));
  bv.setZ(v(2));
  return bv;
}

inline btVector3 toBulletVec3(const double x, const double y, const double z) {
  btVector3 bv;
  bv.setX(x);
  bv.setY(y);
  bv.setZ(z);
  return bv;
}

#endif  // SIMBA_MODELGRAPH_EIGENTOBULLET_H_
