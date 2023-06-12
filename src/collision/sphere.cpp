#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // if within sphere
  Vector3D v = pm.position-this->origin;
  if( v.norm() <= this->radius){
      Vector3D tangent = this->origin + this->radius / v.norm() * v;
      Vector3D corr = tangent - pm.last_position;
      pm.position = (1-this->friction) * corr + pm.last_position;
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
