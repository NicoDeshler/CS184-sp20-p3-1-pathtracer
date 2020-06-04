#ifndef CGL_GLSCENE_AMBIENTLIGHT_H
#define CGL_GLSCENE_AMBIENTLIGHT_H

#include "scene.h"
#include "scene/light.h"

namespace CGL { namespace GLScene {

class AmbientLight : public SceneLight {
 public:

  AmbientLight(const Collada::LightInfo& light_info) {
    this->spectrum = light_info.spectrum;
  }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::InfiniteHemisphereLight* l = 
      new SceneObjects::InfiniteHemisphereLight(spectrum);
    return l;
  }

 private:
  Spectrum spectrum;
};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_AMBIENTLIGHT_H
