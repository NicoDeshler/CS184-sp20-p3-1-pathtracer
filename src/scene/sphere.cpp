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
  
  // Quadratic Equation Coefficients
    double a = (r.d).norm2();
    double b = 2 * dot((r.o - o), r.d);
    double c = (r.o - o).norm2() - r2;
    
    double descriminant = b * b - 4 * a * c;
    double t_minus = r.min_t - 1;
    double t_plus = r.max_t + 1;
    if (descriminant >= 0) {
        t_plus = (-b + sqrt(descriminant)) / (2 * a);
        t_minus = (-b - sqrt(descriminant)) / (2 * a);
    }
    return ((t_minus >= r.min_t) && (t_minus <= r.min_t)) || ((t_plus >= r.min_t) && (t_plus <= r.min_t));
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  // Quadratic Equation Coefficients
    double a = (r.d).norm2();
    double b = 2 * dot((r.o - o), r.d);
    double c = (r.o - o).norm2() - r2;

    // Descriminant
    double descriminant = b * b - 4 * a * c;
    
    // Roots
    double t_minus;
    double t_plus;
    
    // Intersection time variables
    double t_isect;
    bool isIsect = false;
    
    if (descriminant >= 0) {
        t_plus = (-b + sqrt(descriminant)) / (2 * a);
        t_minus = (-b - sqrt(descriminant)) / (2 * a);

        if (t_minus >= r.min_t && t_minus <= r.max_t) {
            // Update intersection variables
            t_isect = t_minus;
            isIsect = true;
            
            // Update ray intersection
            r.max_t = t_isect;

            // Update intersection
            i->t = t_isect;
            i->n = (r.o + t_isect * r.d - o);
            i->n /= i->n.norm();
            i->primitive = this;
            i->bsdf = get_bsdf();
            // if (typeid(ThinFilmBSDF) == typeid(i->bsdf)) {i->n = Vector3D(0,0,1)} // This will make the obect coordinate space constant 
           
        } 
        else if (t_plus >= r.min_t && t_plus <= r.max_t) {
            // Update intersection variables
            t_isect = t_plus;
            isIsect = true;
            
            // Update ray intersection
            r.max_t = t_isect;
            
            // Update intersection
            i->t = t_isect;
            i->n = (r.o + t_isect * r.d - o);
            i->n /= i->n.norm();
            i->primitive = this;
            i->bsdf = get_bsdf();
        }

    }

    return(isIsect); 
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
