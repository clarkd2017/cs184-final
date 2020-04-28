#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

// Forward declarations
class Halfedge;

struct PointMass {
  PointMass(Vector3D position, bool pinned)
      : pinned(pinned), start_position(position), position(position),
        last_position(position) {}

  Vector3D normal();
  /*Vector3D velocity(double delta_t) {
    return (position - last_position) / delta_t;
  }*/

  // static values
  bool pinned;
  Vector3D start_position;

  // dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;

  // mesh reference
  Halfedge *halfedge;

  // FINAL PROJECT: NEW STUFF
  Vector3D predicted_position;
  Vector3D velocity;
  std::vector<PointMass> neighbors;
  float lambda;
  Vector3D delta_p;
  float radius = 0.01; // (constant) particle radius

  // not 100% sure that this works or is necessary
  /*void collide(PointMass p) {

      if ((p.predicted_position - predicted_position).norm() <= 2 * radius) {
          Vector3D temp_p = predicted_position;
          Vector3D temp_v = velocity;
          predicted_position = p.predicted_position;
          velocity = p.velocity;
          p.predicted_position = temp_p;
          p.velocity = temp_v;
      }
  }*/

  float W(PointMass p, float h) {
      return 315.0 / (64.0 * PI * pow(h, 9.0)) * pow(pow(h, 2.0) - pow((position - p.position).norm(), 2.0), 3.0);
  }

  Vector3D gradW(PointMass p, float h) {
      float n = (position - p.position).norm();
      return 45.0 / (PI * pow(h, 6.0)) * pow(h - n, 2.0) / n * (position - p.position);
  }
};

#endif /* POINTMASS_H */
