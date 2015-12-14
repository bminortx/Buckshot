/**
 * The Compound class: Holds all of our compound shapes and constraints,
 * as well as the type of compound we have.
 */

#pragma once

#include <memory>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btAlignedAllocator.h>
#include "../bulletShapes/bullet_cube.h"
#include "../bulletShapes/bullet_cylinder.h"
#include "../bulletShapes/bullet_heightmap.h"
#include "../bulletShapes/bullet_sphere.h"
#include "../bulletShapes/bullet_vehicle.h"

// Useless enum until we get more compounds
enum Compounds{
  VEHICLE = 0
};

class Compound {
public:
  Compound() { }

  Compound(double* Shape_ids, double* Con_ids, Compounds type) {
    shapeid_ = Shape_ids;
    constraintid_ = Con_ids;
    type_ = type;
  }

  double* shapeid_;
  double* constraintid_;
  Compounds type_;
};
