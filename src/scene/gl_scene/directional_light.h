#ifndef CGL_GLSCENE_DIRECTIONALLIGHT_H
#define CGL_GLSCENE_DIRECTIONALLIGHT_H

#include "scene.h"
#include "scene/light.h"

namespace CGL { namespace GLScene {

class DirectionalLight : public SceneLight {
 public:

  DirectionalLight(const Collada::LightInfo& light_info, 
                   const Matrix4x4& transform) {
    this->spectrum = light_info.spectrum;
    this->direction = -(transform * Vector4D(light_info.direction, 1)).to3D();
    this->direction.normalize();
  }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::DirectionalLight* l = 
      new SceneObjects::DirectionalLight(spectrum, direction);
    return l;
  }

 private:

  Spectrum spectrum;
  Vector3D direction;

};

} // namespace GLScene
} // namespace CGL

#endif //CGL_GLSCENE_DIRECTIONALLIGHT_H
