#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  /* ASSUMPTION: Water is a cube of side length side_length
   *
   * struct Particle {
   *    Vector3Df position
   *    Vector3Df last_position
   *    Vector3Df predicted_position
   *    Vector3Df velocity
   *    vector<Particle *> neighbors
   *    float lambda
   *    Vector3Df delta_p
   *    float radius
   * }
   *
   * struct Water {
   *    float side_length
   *    vector<Particle *> particles
   *    vector<vector<int>> pinned
   *    vector<Spring *> springs
   * }
   *
   * float W(Vector3Df r) {
   *    return 315.0 / (64.0 * PI * pow(h, 9.0)) * pow(pow(h, 2.0) - pow(norm(r), 2.0), 3.0)
   * }
   *
   * Vector3Df gradW(Vector3Df r) {
   *    return 45.0 / (PI * pow(h, 6.0)) * pow(h - norm(r), 2.0) / norm(r) * r;
   * }
   *
   * void Plane::collide(Particle &p, float delta_t) {
   *    float t = dot(point - p.last_position, normal) / dot(p.velocity, normal);
   *    if (t >= 0 && t <= delta_t) {
   *        Vector3Df tan_p = p.last_position + t * p.velocity;
   *        Vector3Df ri = p.velocity.unit();
   *        p.velocity = ri - 2 * dot(ri, normal) * normal;
   *        p.predicted_position = tan_p + (delta_t - t) * p.velocity;
   *    }
   * }
   *
   * void Particle::collide(Particle &p) {
   *    // not 100% sure about this
   *    if (norm(p.predicted_position - predicted_position)) <= 2 * particle_radius) {
   *        Vector3Df temp_p = predicted_position;
   *        Vector3Df temp_v = velocity;
   *        predicted_position = p.predicted_position;
   *        velocity = p.velocity;
   *        p.predicted_position = temp_p;
   *        p.velocity = temp_v;
   *    }
   * }
   *
   * User-defined constants:
   *    rho_0 := rest density (kg/m^3)
   *    epsilon := relaxation parameter
   *    h := scale parameter
   *    sigma := normalization factor
   *    d := number of dimensions
   *    solver_iters := number of solver iterations
   *
   * Particle mass: 1.0kg
   * [our own param] Particle radius (particle_radius): 0.01m
   * Kernel radius (h): 0.1m
   * Rest density (rho): 6378.0kg/m^2
   * Density Iterations: 4
   * Time step (dt): 0.0083s (2 substeps of a 60hz frame time)
   * CFM Parameter (epsilon): 600
   * Artificial Pressure Strength (k): 0.0001
   * Artificial Pressure Radius (delta q): 0.03m
   * Artificial Pressure Power (n): 4
   * Artificial Viscosity (c): <= 0.01
   * */
  particles.reserve(rho_0 * rho_0 * rho_0);
  for (float x = 0.0; x < side_length; x += side_length / rho_0) {
      for (float y = 0.0; y < side_length; y += side_length / rho_0) {
          for (float z = 0.0; z < side_length; z += side_length / rho_0) {
              Particle *p = new Particle();
              p->position = Vector3Df(x, y, z);
              p->last_position = p->position;
              particles.emplace_back(p);
          }
      }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    // reset certain Particle fields
    for (auto *p: particles) {
        p->predicted_position *= 0.0;
        p->neighbors.clear();
        p->lambda = 0.0;
        p->delta_p *= 0.0;
    }

    // apply forces to update position
    double delta_t = 1.0f / frames_per_sec / simulation_steps;
    Vector3Df f_ext = Vector3Df();
    for (auto a: external_accelerations) {
        f_ext += a;
    }
    for (auto *p: particles) {
        p->velocity += delta_t * f_ext;
        p->predicted_position = p->position + delta_t * p->velocity;
    }

    // TODO: find neighboring particles
    // construct KD tree
    cv::Mat M(rho_0 * rho_0 * rho_0, 3, CV_64F);

    for (auto *p: particles) {
        p->neighbors = ;
    }

    // solver loop
    for (unsigned it = 0; it < solver_iters; it++) {
        // calculate lambda_i
        for (auto *p_i: particles) {
            float rho_i = 0.0;
            for (auto *p_j: p_i->neighbors) {
                rho_i += mass * W(p_i->position - p_j->position);
            }
            float C_i = rho_i / rho_0 - 1.0;
            float grad_sum = 0.0;
            for (auto *p_k: particles) {
                Vector3Df grad_pk_Ci = Vector3DF();
                if (p_k == p_i) {
                    for (auto *p_j: p_i->neighbors) {
                        grad_pk_Ci += gradW(p_i->position - p_j->position);
                    }
                } else {
                    // not 100% sure about this
                    grad_pk_Ci = -gradW(p_i->position - p_k->position);
                }
                grad_pk_Ci /= rho_0;
                grad_sum += pow(norm(grad_pk_Ci), 2.0);
            }
            p_i->lambda = -C_i / (grad_sum + epsilon);
        }

        for (auto *p_i: particles) {
            // calculate s_corr (artificial pressure term) and delta_p_i
            for (auto *p_j: p_i->neighbors) {
                float s_corr = -k * pow(W(p_i->position - p_j->position) / W(delta_q), (float) n);
                p_i->delta_p += (p_i->lambda + p_j->lambda + s_corr) * gradW(p_i->position - p_j->position);
            }
            p_i->delta_p /= rho_0;

            // collide with other particles
            for (auto *p_j: p_i->neighbors) {
                p_i->collide(*p_j)
            }

            // collide with objects (planes)
            for (auto obj: *collision_objects) {
                obj->collide(*p_i);
            }
        }

        // update predicted positions
        for (auto *p_i: particles) {
            p_i->predicted_position += p_i->delta_p;
        }
    }

    for (auto *p_i: particles) {
        // update velocity
        p_i->velocity = 1.0 / delta_t * (p_i->predicted_position - p_i->position);

        // TODO: vorticity confinement

        // TODO: viscosity constraint

        // update position
        p_i->last_position =  p_i->position;
        p_i->position = p_i->predicted_position;
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
