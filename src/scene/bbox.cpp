#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
    Vector3D x_hat(1, 0, 0);
    double tx0 = dot((min - r.o), x_hat) / dot(r.d, x_hat);
    double tx1 = dot((max - r.o), x_hat) / dot(r.d, x_hat);
    if (tx1 < tx0) std::swap(tx0, tx1);

    Vector3D y_hat(0, 1, 0);
    double ty0 = dot((min - r.o), y_hat) / dot(r.d, y_hat);
    double ty1 = dot((max - r.o), y_hat) / dot(r.d, y_hat);
    if (ty1 < ty0) std::swap(ty0, ty1);

    Vector3D z_hat(0, 0, 1);
    double tz0 = dot((min - r.o), z_hat) / dot(r.d, z_hat);
    double tz1 = dot((max - r.o), z_hat) / dot(r.d, z_hat);
    if (tz1 < tz0) std::swap(tz0, tz1);

    double t_min = std::max(tx0, ty0);
    double t_max = std::min(tx1, ty1);
    if (t_min > t_max) return false;

    if (tz0 > t_max || tz1 < t_min) return false;
    t_min = std::max(tz0, t_min);
    t_max = std::min(tz1, t_max);

    if (t_min > t_max) return false;

    // overlap between ray and box intersection interval
    if (!((t1 - t_min) >= 0 && (t_max - t0) >= 0)) return false;
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
