#include "sphere.h"

#include "scene/object.h"
#include "util/sphere_drawing.h"

#include "pathtracer/bsdf.h"

namespace CGL { namespace GLScene {

Sphere::Sphere(const Collada::SphereInfo& info, 
               const Vector3D& position, const double scale) : 
  p(position), r(info.radius * scale) { 
  if (info.material) {
    bsdf = info.material->bsdf;
  } else {
    bsdf = new DiffuseBSDF(Spectrum(0.5f,0.5f,0.5f));    
  }
}

void Sphere::set_draw_styles(DrawStyle *defaultStyle, DrawStyle *hoveredStyle,
                             DrawStyle *selectedStyle) {
  style = defaultStyle;
}

void Sphere::render_in_opengl() const {
  Misc::draw_sphere_opengl(p, r);
}

BBox Sphere::get_bbox() {
  return BBox(p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r);
}

BSDF* Sphere::get_bsdf() {
  return bsdf;
}

SceneObjects::SceneObject *Sphere::get_static_object() {
  return new SceneObjects::SphereObject(p, r, bsdf);
}

} // namespace GLScene
} // namespace CGL
