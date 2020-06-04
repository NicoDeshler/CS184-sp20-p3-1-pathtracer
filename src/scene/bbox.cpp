#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

    // yz-planes
    Vector3D x_hat(1, 0, 0);
    double tx0 = dot((min - r.o), x_hat) / dot(r.d, x_hat);
    double tx1 = dot((max - r.o), x_hat) / dot(r.d, x_hat);
    if (tx1 < tx0) std::swap(tx0, tx1);

    // xz-planes
    Vector3D y_hat(0, 1, 0);
    double ty0 = dot((min - r.o), y_hat) / dot(r.d, y_hat);
    double ty1 = dot((max - r.o), y_hat) / dot(r.d, y_hat);
    if (ty1 < ty0) std::swap(ty0, ty1); 

    // xy-planes
    Vector3D z_hat(0, 0, 1);
    double tz0 = dot((min - r.o), z_hat) / dot(r.d, z_hat);
    double tz1 = dot((max - r.o), z_hat) / dot(r.d, z_hat);
    if (tz1 < tz0) std::swap(tz0, tz1);

    // Check rectangle intersection along a single z-slice first 
    double t_min = std::max(tx0, ty0);
    double t_max = std::min(tx1, ty1);
    if (t_min > t_max) return false;

    // If still a contender, check intersection points along z planes
    if (tz0 > t_max || tz1 < t_min) return false;
    t_min = std::max(tz0, t_min);
    t_max = std::min(tz1, t_max);
    
    // Check that entrance (t_min) and exit (t_max) times make sense.
    if (t_min > t_max) return false;

    // Check that there is overlap between the ray interval and the box interesection interval.
    if (!((t1 - t_min) >= 0 && (t_max - t0) >= 0)) return false;
    
    // All cases checked so return that here was indeed an intersection with a box.
    return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
