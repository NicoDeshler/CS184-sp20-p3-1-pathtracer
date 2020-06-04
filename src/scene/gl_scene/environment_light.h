#ifndef CGL_GLSCENE_ENVIRONMENTLIGHT_H
#define CGL_GLSCENE_ENVIRONMENTLIGHT_H

#include "scene.h"
#include "util/image.h"
#include "scene/light.h"

namespace CGL { namespace GLScene {

class EnvironmentLight : public SceneLight {
 public:

  EnvironmentLight(HDRImageBuffer* envmap) : envmap(envmap) { }

  SceneObjects::SceneLight *get_static_light() const {
    SceneObjects::EnvironmentLight* l = 
      new SceneObjects::EnvironmentLight(envmap);
    return l;
  }

 private:

  HDRImageBuffer* envmap;

};

} // namespace GLScene
} // namespace CGL

#endif // CGL_GLSCENE_ENVIRONMENTLIGHT_H
