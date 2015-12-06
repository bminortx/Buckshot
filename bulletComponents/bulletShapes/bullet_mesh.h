#ifndef BULLET_MESH_H
#define BULLET_MESH_H

#include <bullet/btBulletDynamicsCommon.h>
//#include <bullet/BulletCollision/CollisionShapes/bt.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include <ModelGraph/Bullet_shapes/bullet_shape.h>

// Constructs a collision shape from an imported mesh

class bullet_mesh: public bullet_shape{

 public:
  // construction
  explicit bullet_mesh(const std::shared_ptr<ModelNode>& mnMesh) {
    std::shared_ptr<MeshShape> pMesh =
        std::dynamic_pointer_cast<MeshShape>(mnMesh);
    const aiScene *pScene = aiImportFile(pMesh->GetFileDir().c_str() ,
                                         aiProcess_Triangulate |
                                         aiProcess_FindDegenerates |
                                         aiProcess_GenSmoothNormals |
                                         aiProcess_JoinIdenticalVertices |
                                         aiProcess_OptimizeMeshes |
                                         aiProcess_FindInvalidData |
                                         aiProcess_FixInfacingNormals);
    btVector3 dMin(DBL_MAX, DBL_MAX, DBL_MAX);
    btVector3 dMax(DBL_MIN, DBL_MIN, DBL_MIN);
    btTriangleMesh* pTriangleMesh = new btTriangleMesh(true, true);
    GenerateStaticHull(pScene, pScene->mRootNode,
                       pScene->mRootNode->mTransformation,
                       1.0, *pTriangleMesh, dMin, dMax);
    bulletShape = std::make_shared<btBvhTriangleMeshShape>(
        pTriangleMesh, true, true);
    bulletMotionState = std::make_shared<NodeMotionState>(mnMesh);
    btRigidBody::btRigidBodyConstructionInfo cInfo(pMesh->GetMass(),
                                                   bulletMotionState.get(),
                                                   bulletShape.get(),
                                                   btVector3(0, 0, 0));
    bulletBody = std::make_shared<btRigidBody>(cInfo);
    bulletBody->setContactProcessingThreshold(BT_LARGE_FLOAT);
  }

  // To actually get the mesh, we have to connect the dots...

  void GenerateStaticHull(const struct aiScene *pAIScene,
                          const struct aiNode *pAINode,
                          const aiMatrix4x4 parentTransform,
                          const float flScale,
                          btTriangleMesh &triangleMesh ,
                          btVector3 &dMin, btVector3 &dMax) {
    aiMesh *pAIMesh;
    aiFace *pAIFace;
    for (size_t x = 0; x < pAINode->mNumMeshes; x++) {
      pAIMesh = pAIScene->mMeshes[pAINode->mMeshes[x]];
      for (size_t y = 0; y < pAIMesh->mNumFaces; y++) {
        pAIFace = &pAIMesh->mFaces[y];
        if (pAIFace->mNumIndices!= 3) {
          //s          cout<<pAIFace->mNumIndices<<endl;
        }
        else {
          aiVector3D v1 = parentTransform
              * pAIMesh->mVertices[pAIFace->mIndices[0]];
          aiVector3D v2 = parentTransform
              * pAIMesh->mVertices[pAIFace->mIndices[1]];
          aiVector3D v3 = parentTransform
              * pAIMesh->mVertices[pAIFace->mIndices[2]];
          dMin[0] = std::min((float)dMin[0],
                             std::min(v1.x, std::min(v2.x, v3.x)));
          dMax[0] = std::max((float)dMax[0],
                             std::min(v1.x, std::max(v2.x, v3.x)));
          dMin[1] = std::min((float)dMin[1],
                             std::min(v1.y, std::min(v2.y, v3.y)));
          dMax[1] = std::max((float)dMax[1],
                             std::max(v1.y, std::max(v2.y, v3.y)));
          dMin[2] = std::min((float)dMin[2],
                             std::min(v1.z, std::min(v2.z, v3.z)));
          dMax[2] = std::max((float)dMax[2],
                             std::max(v1.z, std::max(v2.z, v3.z)));
          triangleMesh.addTriangle(
              btVector3(v1.x * flScale, v1.y * flScale, v1.z * flScale),
              btVector3(v2.x * flScale, v2.y * flScale, v2.z * flScale),
              btVector3(v3.x * flScale, v3.y * flScale, v3.z * flScale),
              false);
        }
        //     assert(pAIFace->mNumIndices == 3);
      }
    }
    for (size_t x = 0; x < pAINode->mNumChildren; x++) {
      GenerateStaticHull(pAIScene, pAINode->mChildren[x],
                         parentTransform*pAINode->mChildren[x]->mTransformation,
                         flScale, triangleMesh, dMin, dMax);
    }
  }

  void set_vertex_data() {}
};

#endif //  BULLET_MESH_H
