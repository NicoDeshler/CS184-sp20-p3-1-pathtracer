#ifndef CGL_RANDOMUTIL_H
#define CGL_RANDOMUTIL_H

#include <random>

#define XORSHIFT_RAND

namespace CGL {

#ifndef XORSHIFT_RAND

#ifdef __GNUC__
static std::mersenne_twister_engine<std::uint_fast32_t, 32, 624, 397, 31, 0x9908b0df,
                             11, 0xffffffff, 7, 0x9d2c5680, 15, 0xefc60000, 18,
                             1812433253> minstd_engine;

static float rmax = 1.0 / (minstd_engine.max() - minstd_engine.min());
#endif

#else
#include <stdint.h>

static struct xorshift32_state {
  uint32_t a = 0x12345678;
} rng_state;

const static double inv_uint32_max = 1.0 / (double) UINT32_MAX;

/* The state word must be initialized to non-zero */
inline uint32_t xorshift32(xorshift32_state &state) {
  /* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
  uint32_t x = state.a;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return state.a = x;
}
#endif

/**
 * Returns a number distributed uniformly over [0, 1].
 */
inline double random_uniform() {
#ifdef XORSHIFT_RAND
  return (double)xorshift32(rng_state) * inv_uint32_max;
#else
#ifdef __GNUC__
  return (double)(minstd_engine() - minstd_engine.min()) * rmax;
#else
  return (double)std::rand() / RAND_MAX;
#endif
#endif
}

/**
 * Returns true with probability p and false with probability 1 - p.
 */
inline bool coin_flip(double p) {
  return random_uniform() < p;
}

} // namespace CGL

#endif  // CGL_RANDOMUTIL_H
