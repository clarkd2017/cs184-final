#ifndef WATER_H
#define WATER_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/collisionObject.h"

using namespace CGL;
using namespace std;

struct WaterParameters {
  WaterParameters() {}
  WaterParameters(double density)
      : density(density) {}
  ~WaterParameters() {}

  // Global simulation parameters

  //double damping;

  // Mass-spring parameters
  double density;
};

struct Water {
  Water() {}
  Water(double width, double height, double depth, double density);
  ~Water();

  void buildVolume();

  void simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  //void buildWaterMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Water properties
  double width;
  double height;
  double depth;
  double density;

  // Water components
  vector<PointMass> point_masses;
  double p_mass;
  //WaterMesh *waterMesh;

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
};

#endif /* WATER_H */
