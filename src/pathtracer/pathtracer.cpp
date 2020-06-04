#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out(0.0);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  Vector3D wi_d;
  float pdf;
  for (int s = 0; s < num_samples; s++) {
      Spectrum f = isect.bsdf->sample_f(w_out, &wi_d, &pdf);    // Get the bsdf for the first hit surface (sets wi_d to a random hemisphere direction and the pdf based on the sampling scheme)
      float cosTheta = dot(wi_d, Vector3D(0, 0, 1)); // dot product with normal vector in object space
      
      // Set variables for next intersection (for one bounce, the next intersection must be a light source)
      Vector3D next_d = o2w * wi_d;
      Vector3D next_o = hit_p; 
      
      Ray r_next = Ray(next_o, next_d, 1);
      r_next.min_t = EPS_F;
      Intersection i_next;
      if (cosTheta > 0 && bvh->intersect(r_next, &i_next)) {
          Spectrum L = i_next.bsdf->get_emission();
          L_out += L * f * cosTheta / pdf;
      }
  }
  L_out /= num_samples;
  return L_out;
}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  for (auto light = scene->lights.begin(); light != scene->lights.end(); light++) {
      //SceneLight::sample_L(Vector3D& p, Vector3D* wi, float* distToLight, float* pdf),
      Vector3D wi_d;
      float distToLight;
      float pdf;
      float cosTheta;
      int num_samples = 0;
      Spectrum f;
      Spectrum L;
      Spectrum L_light;

      for (int s = 0; s < ns_area_light; s++) {
          L = (*light)->sample_L(hit_p, &wi_d, &distToLight, &pdf);
          cosTheta = dot(wi_d, isect.n);


          // Set variables for next intersection (for one bounce, the next intersection must be a light source)
          Vector3D next_d = wi_d;
          Vector3D next_o = hit_p;

          Ray r_next = Ray(next_o, next_d, 1);
          r_next.min_t = EPS_F;
          r_next.max_t = (double)distToLight - EPS_F;
          
          // check for obstructions
          Intersection i_next;
          if (cosTheta > 0 && !(bvh->intersect(r_next, &i_next))) {
              f = isect.bsdf->f(w_out, wi_d); //Just gets the hit surface BSDF
              double r2 = (double)distToLight * (double)distToLight;
              L_light += L * f * cosTheta / pdf;

          }
          num_samples++;                    
          
          if ((*light)->is_delta_light()) {
              break;
          }
      }
      L_out += L_light / num_samples;
  
  }

  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
    
    return isect.bsdf->get_emission();
    
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`


    if (direct_hemisphere_sample) {
        return estimate_direct_lighting_hemisphere(r, isect);
    }
    else {
        return estimate_direct_lighting_importance(r, isect);
    }
    
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Spectrum L_out(0, 0, 0);

  int num_samples = scene->lights.size()* ns_area_light;

  // Direct illumination on recursed object
  L_out += one_bounce_radiance(r, isect);


  // Indirect Path Trace
  double cpdf = 0.65; // Russian Roulette ray continuation probability
  if (coin_flip(cpdf)) {

        Vector3D wi_d;
        float pdf;
      
        Spectrum f = isect.bsdf->sample_f(w_out, &wi_d, &pdf);    // Get the bsdf for the first hit surface (sets wi_d to a random hemisphere direction and the pdf based on the sampling scheme)
        float cosTheta = dot(wi_d, Vector3D(0, 0, 1)); // dot product with normal vector in object space

        // Set ray parameters variables for next intersection
        Vector3D next_d = o2w * wi_d;
        Vector3D next_o = hit_p;
        
        // Make the next random ray
        Ray r_next = Ray(next_o, next_d);
        r_next.depth = r.depth - 1;
        r_next.min_t = EPS_F;
        Intersection i_next;
        
        if (r_next.depth > 1 && cosTheta > 0 && bvh->intersect(r_next, &i_next)) {
            Spectrum L = at_least_one_bounce_radiance(r_next, i_next);
            L_out += L * f * cosTheta / pdf / cpdf;
        }
    
  }
  
  return L_out;
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.
  
  if (!bvh->intersect(r, &isect))
    return L_out;

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  
  // TODO (Part 3): Return the direct illumination.

  L_out = zero_bounce_radiance(r, isect);
  L_out += at_least_one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.


  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"



  int num_samples = ns_aa;          // max number of samples per pixel to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Spectrum radiance(0.);
  Spectrum s_radiance;
  float s1 = 0;
  float s2 = 0;
  float var;
  float mu2;
  float mu;
  int batchSize = 0;
  int n = 0;


  for (int i = 0; i < num_samples; i++, n++, batchSize++) {
      // Adaptive sampling check
      if (batchSize == samplesPerBatch) {
          batchSize = 0;
          mu = s1 / (float)n;
          mu2 = mu * mu;
          var = (1 / ((float)n - 1)) * (s2 - (s1 * s1) / (float)n);
          float I = 1.96 * 1.96 * var / (float)n;
          float cvoff = maxTolerance * maxTolerance * mu2; // convergence cutoff
          
          if (I <= cvoff) {
              break;
          }
           
      }
      
      Vector2D rs = gridSampler->get_sample();         // random offset within pixel
      Vector2D ps = origin + rs;                       // pixel sample
      Ray r = camera->generate_ray(ps.x/sampleBuffer.w, ps.y/sampleBuffer.h);   // Ray in world space
      r.depth = max_ray_depth;
      s_radiance = est_radiance_global_illumination(r);
      float s_illum = s_radiance.illum();
      s1 += s_illum;
      s2 += s_illum * s_illum;
      radiance += s_radiance;

  }
  radiance /= n;
  sampleBuffer.update_pixel(Spectrum(radiance.x, radiance.y, radiance.z), x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = n;
}

} // namespace CGL