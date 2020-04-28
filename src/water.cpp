#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "water.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Water::Water(double width, double height, double depth, double density) {
  this->width = width;
  this->height = height;
  this->depth = depth;
  this->density = density;

  buildVolume();
}

Water::~Water() {
  point_masses.clear();
}

void Water::buildVolume() {
  double x_interval = 0.02;
  double y_interval = 0.02;
  double z_interval = 0.02;
  /*int num_height_points = floor(height / y_interval);
  int num_width_points = floor(width / x_interval);
  int num_depth_points = floor(depth / z_interval);
  double tot_mass = density * (num_width_points * num_height_points * num_depth_points) * 0.000008;
  p_mass = tot_mass / (num_width_points * num_height_points * num_depth_points);

  for (int h = 0; h < num_height_points; h++){
    for (int w = 0; w < num_width_points; w++){
      for (int d = 0; d < num_depth_points; d++){
        double x = x_interval * w;
        double y = y_interval * h;
        double z = z_interval * d;

        PointMass pm = PointMass(Vector3D(x, y, z), false);
        point_masses.emplace_back(pm);
      }
    }
  }*/
  point_masses.emplace_back(PointMass(Vector3D(0.5, 0.2, 0.5), false));
}

void Water::simulate(double frames_per_sec, double simulation_steps, WaterParameters *wp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    // temporarily (permanently?) define some constants
    unsigned solver_iters = 4;
    float particle_mass = 1.0;
    float rho_0 = 6378.0;
    float epsilon = 600;
    float k = 0.0001;
    float h = 0.1;
    float delta_q = 0.03;
    float Wdq = 315.0 / (64.0 * PI * pow(h, 9.0)) *  pow(pow(h, 2.0) - pow(delta_q, 2.0), 3.0);
    float n = 4.0;

    // apply forces to update position
    double delta_t = 1.0f / frames_per_sec / simulation_steps;
    Vector3D f_ext = Vector3D();
    for (auto a: external_accelerations) {
        f_ext += a;
    }
    for (auto &p: point_masses) {
        p.velocity += delta_t * f_ext;
        p.predicted_position = p.position + delta_t * p.velocity;
        p.delta_t = delta_t;
    }

    // TODO: find neighboring particles + populate each PM's neighbors attribute

    // should put the below into this loop but moving it out for now
    //for (unsigned it = 0; it < solver_iters; it++) {
    //}

    for (auto &p_i: point_masses) {
        // calculate lambda_i
        float rho_i = 0.0;
        for (auto &p_j: p_i.neighbors) {
            rho_i += particle_mass * p_i.W(p_j, h);
        }
        float C_i = rho_i / rho_0 - 1.0;

        float grad_sum = 0.0;
        for (auto &p_k: p_i.neighbors) {
            Vector3D grad_pk_Ci = Vector3D();
            if (&p_k == &p_i) {
                for (auto &p_j: p_i.neighbors) {
                    grad_pk_Ci += p_i.gradW(p_j, h);
                }
            } else {
                grad_pk_Ci = -1.0 * p_i.gradW(p_k, h);
            }
            grad_pk_Ci /= rho_0;
            grad_sum += pow(grad_pk_Ci.norm(), 2.0);
        }
        p_i.lambda = -1.0 * C_i / (grad_sum + epsilon);
    }

    for (auto &p_i: point_masses) {
        // calculate s_corr (artificial pressure term) and delta_p_i
        for (auto &p_j: p_i.neighbors) {
            float s_corr = -k * pow(p_i.W(p_j, h) / Wdq, n);
            p_i.delta_p += (p_i.lambda + p_j.lambda + s_corr) * p_i.gradW(p_j, h);
        }
        p_i.delta_p /= rho_0;

        // collide with other particles (may not be necessary)
        /*for (auto &p_j: p_i.neighbors) {
            p_i.collide(p_j);
        }*/

        // collide with objects (planes)
        for (auto obj: *collision_objects) {
            obj->collide(p_i);
        }
    }

    for (auto &p_i: point_masses) {
        // update predicted positions
        p_i.predicted_position += p_i.delta_p;
        // NOTE: if solver_iters put back in, then the below should be in a separate loop
        // update velocity
        p_i.velocity = 1.0 / delta_t * (p_i.predicted_position - p_i.position);

        // TODO: vorticity confinement

        // TODO: viscosity constraint

        // update position
        p_i.last_position =  p_i.position;
        p_i.position = p_i.predicted_position;

        // reset certain PointMass attributes
        p_i.predicted_position *= 0.0;
        p_i.neighbors.clear();
        p_i.lambda = 0.0;
        p_i.delta_p *= 0.0;
    }
}

void Water::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Water::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Water::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Water::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

// void Water::buildWaterMesh() {
//   if (point_masses.size() == 0) return;

//   ClothMesh *clothMesh = new ClothMesh();
//   vector<Triangle *> triangles;

//   // Create vector of triangles
//   for (int y = 0; y < num_height_points - 1; y++) {
//     for (int x = 0; x < num_width_points - 1; x++) {
//       PointMass *pm = &point_masses[y * num_width_points + x];
//       // Get neighboring point masses:
//       /*                      *
//        * pm_A -------- pm_B   *
//        *             /        *
//        *  |         /   |     *
//        *  |        /    |     *
//        *  |       /     |     *
//        *  |      /      |     *
//        *  |     /       |     *
//        *  |    /        |     *
//        *      /               *
//        * pm_C -------- pm_D   *
//        *                      *
//        */
      
//       float u_min = x;
//       u_min /= num_width_points - 1;
//       float u_max = x + 1;
//       u_max /= num_width_points - 1;
//       float v_min = y;
//       v_min /= num_height_points - 1;
//       float v_max = y + 1;
//       v_max /= num_height_points - 1;
      
//       PointMass *pm_A = pm                       ;
//       PointMass *pm_B = pm                    + 1;
//       PointMass *pm_C = pm + num_width_points    ;
//       PointMass *pm_D = pm + num_width_points + 1;
      
//       Vector3D uv_A = Vector3D(u_min, v_min, 0);
//       Vector3D uv_B = Vector3D(u_max, v_min, 0);
//       Vector3D uv_C = Vector3D(u_min, v_max, 0);
//       Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
//       // Both triangles defined by vertices in counter-clockwise orientation
//       triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
//                                        uv_A, uv_C, uv_B));
//       triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
//                                        uv_B, uv_C, uv_D));
//     }
//   }

//   // For each triangle in row-order, create 3 edges and 3 internal halfedges
//   for (int i = 0; i < triangles.size(); i++) {
//     Triangle *t = triangles[i];

//     // Allocate new halfedges on heap
//     Halfedge *h1 = new Halfedge();
//     Halfedge *h2 = new Halfedge();
//     Halfedge *h3 = new Halfedge();

//     // Allocate new edges on heap
//     Edge *e1 = new Edge();
//     Edge *e2 = new Edge();
//     Edge *e3 = new Edge();

//     // Assign a halfedge pointer to the triangle
//     t->halfedge = h1;

//     // Assign halfedge pointers to point masses
//     t->pm1->halfedge = h1;
//     t->pm2->halfedge = h2;
//     t->pm3->halfedge = h3;

//     // Update all halfedge pointers
//     h1->edge = e1;
//     h1->next = h2;
//     h1->pm = t->pm1;
//     h1->triangle = t;

//     h2->edge = e2;
//     h2->next = h3;
//     h2->pm = t->pm2;
//     h2->triangle = t;

//     h3->edge = e3;
//     h3->next = h1;
//     h3->pm = t->pm3;
//     h3->triangle = t;
//   }

//   // Go back through the cloth mesh and link triangles together using halfedge
//   // twin pointers

//   // Convenient variables for math
//   int num_height_tris = (num_height_points - 1) * 2;
//   int num_width_tris = (num_width_points - 1) * 2;

//   bool topLeft = true;
//   for (int i = 0; i < triangles.size(); i++) {
//     Triangle *t = triangles[i];

//     if (topLeft) {
//       // Get left triangle, if it exists
//       if (i % num_width_tris != 0) { // Not a left-most triangle
//         Triangle *temp = triangles[i - 1];
//         t->pm1->halfedge->twin = temp->pm3->halfedge;
//       } else {
//         t->pm1->halfedge->twin = nullptr;
//       }

//       // Get triangle above, if it exists
//       if (i >= num_width_tris) { // Not a top-most triangle
//         Triangle *temp = triangles[i - num_width_tris + 1];
//         t->pm3->halfedge->twin = temp->pm2->halfedge;
//       } else {
//         t->pm3->halfedge->twin = nullptr;
//       }

//       // Get triangle to bottom right; guaranteed to exist
//       Triangle *temp = triangles[i + 1];
//       t->pm2->halfedge->twin = temp->pm1->halfedge;
//     } else {
//       // Get right triangle, if it exists
//       if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
//         Triangle *temp = triangles[i + 1];
//         t->pm3->halfedge->twin = temp->pm1->halfedge;
//       } else {
//         t->pm3->halfedge->twin = nullptr;
//       }

//       // Get triangle below, if it exists
//       if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
//         Triangle *temp = triangles[i + num_width_tris - 1];
//         t->pm2->halfedge->twin = temp->pm3->halfedge;
//       } else {
//         t->pm2->halfedge->twin = nullptr;
//       }

//       // Get triangle to top left; guaranteed to exist
//       Triangle *temp = triangles[i - 1];
//       t->pm1->halfedge->twin = temp->pm2->halfedge;
//     }

//     topLeft = !topLeft;
//   }

//   clothMesh->triangles = triangles;
//   this->clothMesh = clothMesh;
// }
