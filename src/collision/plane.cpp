#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../leak_fix.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void collideFace(PointMass &pm, Vector3D point, Vector3D normal, float friction) {
    // check for collision with 1 plane/face of "tank", correct point mass position if necessary
    Vector3D displaced_point = point + pm.radius * normal;
    float t = dot(displaced_point - pm.last_position, normal) / dot(pm.velocity, normal);
    if (t >= 0 && t <= pm.delta_t) {
//        std::cout << normal;

        pm.collision = true;
        Vector3D tan_p = pm.last_position + t * pm.velocity + SURFACE_OFFSET * normal;
        pm.velocity -= 2.0 * dot(pm.velocity, normal) * normal;
        pm.velocity *= 1.0 - friction;
        pm.predicted_position = tan_p;
    }
}

void Plane::collide(PointMass &pm) {
    // get points and normals for 4 other faces, collide with all faces (invisible boundaries)
  
    collideFace(pm, point, normal, friction); //bottom face/plane
    collideFace(pm, point + Vector3D(-1.0f, 0.0f, 0.0f), Vector3D(1.0f,0.0f,0.0f), friction); //right plane
    collideFace(pm, point + Vector3D(1.0f, 0.0f, 0.0f), Vector3D(-1.0f,0.0f,0.0f), friction); //left plane
    collideFace(pm, point + Vector3D(0.0f, 0.0f, -1.0f), Vector3D(0.0f,0.0f,1.0f), friction); //front plane
    collideFace(pm, point + Vector3D(0.0f, 0.0f, 1.0f), Vector3D(0.0f,0.0f,-1.0f), friction); //back plane
    
//    collideFace(pm, point + Vector3D(-0.5f, 0.0f, 0.0f), Vector3D(1.0f,0.0f,0.0f), friction); //right plane
//    collideFace(pm, point + Vector3D(1.5f, 0.0f, 0.0f), Vector3D(-1.0f,0.0f,0.0f), friction); //left plane
//    collideFace(pm, point + Vector3D(0.0f, 0.0f, -0.5f), Vector3D(0.0f,0.0f,1.0f), friction); //front plane
//    collideFace(pm, point + Vector3D(0.0f, 0.0f, 1.5f), Vector3D(0.0f,0.0f,-1.0f), friction); //back plane
    collideFace(pm, point + Vector3D(0.0f, 3.0f, 0.0f), -normal, friction);
}

void Plane::render(GLShader &shader) {
  
  nanogui::Color color(0.6f, 0.5f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 1 * (sCross + sParallel);
  positions.col(1) << sPoint + 1 * (sCross - sParallel);
  positions.col(2) << sPoint + 1 * (-sCross + sParallel);
  positions.col(3) << sPoint + 1 * (-sCross - sParallel);
//    std::cout << positions;
    
  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }
  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);

    
// attempt at open cube
    
//  MatrixXf positions(3, 21);
    
//    positions.col(0) << -1.0f,-1.0f,-1.0f; // triangle 1 : begin
//    positions.col(1) << -1.0f,-1.0f, 1.0f;
//    positions.col(2) <<-1.0f, 1.0f, 1.0f; // triangle 1 : end
    
//    positions.col(0) <<1.0f, 1.0f,-1.0f; // triangle 2 : begin
//    positions.col(1) <<-1.0f,-1.0f,-1.0f;
//    positions.col(2) <<-1.0f, 1.0f,-1.0f; // triangle 2 : end
//    positions.col(3) <<1.0f,-1.0f, 1.0f;
//    positions.col(4) <<-1.0f,-1.0f,-1.0f;
//    positions.col(5) <<1.0f,-1.0f,-1.0f;
//    positions.col(6) <<1.0f, 1.0f,-1.0f;
//    positions.col(7) <<1.0f,-1.0f,-1.0f;
//    positions.col(8) <<-1.0f,-1.0f,-1.0f;
//    positions.col(9) <<-1.0f,-1.0f,-1.0f;
//    positions.col(10) <<-1.0f, 1.0f, 1.0f;
//    positions.col(11) <<-1.0f, 1.0f,-1.0f;
//    positions.col(12) <<1.0f,-1.0f, 1.0f;
//    positions.col(13) <<-1.0f,-1.0f, 1.0f;
//    positions.col(14) <<-1.0f,-1.0f,-1.0f;
//    positions.col(15) <<-1.0f, 1.0f, 1.0f;
//    positions.col(16) <<-1.0f,-1.0f, 1.0f;
//    positions.col(17) <<1.0f,-1.0f, 1.0f;
//    positions.col(18) <<1.0f, 1.0f, 1.0f;
//    positions.col(19) <<1.0f,-1.0f,-1.0f;
//    positions.col(20) <<1.0f, 1.0f,-1.0f;
    
//    positions.col(24) <<1.0f,-1.0f,-1.0f;
//    positions.col(25) <<1.0f, 1.0f, 1.0f;
//    positions.col(26) <<1.0f,-1.0f, 1.0f;

//  shader.uploadAttrib("in_position", positions);
//  shader.drawArray(GL_TRIANGLE_STRIP, 0, 27);
    
#ifdef LEAK_PATCH_ON
  shader.freeAttrib("in_position");
  if (shader.attrib("in_normal", false) != -1) {
    shader.freeAttrib("in_normal");
  }
#endif
}
