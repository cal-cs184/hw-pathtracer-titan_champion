#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
 
    // Vector from ray origin to sphere center
    Vector3D oc = r.o - o;

    double a = dot(r.d, r.d);
    double b = 2 * dot(oc, r.d);
    double c = dot(oc, oc) - r2;  // r2 = radius squared

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;  // no intersection

    double sqrt_disc = sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2 * a);
    double t2 = (-b + sqrt_disc) / (2 * a);

    // Check if either intersection is within the valid t range
    if (t1 >= r.min_t && t1 <= r.max_t) {
        r.max_t = t1;
        return true;
    }
    if (t2 >= r.min_t && t2 <= r.max_t) {
        r.max_t = t2;
        return true;
    }

    return false;
   
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

    Vector3D oc = r.o - o;

    double a = dot(r.d, r.d);
    double b = 2 * dot(oc, r.d);
    double c = dot(oc, oc) - r2;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;

    double sqrt_disc = sqrt(discriminant);
    double t1 = (-b - sqrt_disc) / (2 * a);
    double t2 = (-b + sqrt_disc) / (2 * a);

    double t = -1;

    if (t1 >= r.min_t && t1 <= r.max_t) {
        t = t1;
    }
    else if (t2 >= r.min_t && t2 <= r.max_t) {
        t = t2;
    }
    else {
        return false;
    }

    r.max_t = t;

    // Update intersection record
    i->t = t;
    Vector3D p = r.o + t * r.d;
    i->n = (p - o).unit();  
    i->primitive = this;
    i->bsdf = get_bsdf();

    return true;
  
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
