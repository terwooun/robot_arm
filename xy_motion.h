#ifndef ROBOT_ARM_XY_MOTION_H
#define ROBOT_ARM_XY_MOTION_H

#include <float.h>
#include <math.h>

// Geometry only: no Arduino state, pulse output, homing, or motor direction.
// Both angles are ABSOLUTE physical link orientations, measured CCW from
// the forward horizontal in the arm's vertical plane. a is base -> elbow
// (Y motor); b is elbow -> wrist (X motor), not the relative elbow angle.
namespace XYMotion {

constexpr float LINK_1_MM = 120.0f;
constexpr float LINK_2_MM = 120.0f;
constexpr float MIN_BEND_DEG = 10.0f;
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr float DEGREES_TO_RADIANS = PI_VALUE / 180.0f;
constexpr float RADIANS_TO_DEGREES = 180.0f / PI_VALUE;
constexpr float MIN_BEND_SIN = 0.17364817766693033f;

struct Pose {
  float a;
  float b;
};

struct Point {
  float forward;
  float up;
};

inline bool finiteNumber(float value) {
  return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

inline float wrap180(float angle) {
  if (!finiteNumber(angle)) return NAN;
  float wrapped = fmodf(angle, 360.0f);
  if (wrapped >= 180.0f) wrapped -= 360.0f;
  if (wrapped < -180.0f) wrapped += 360.0f;
  return wrapped;
}

// Select the equivalent orientation nearest the previous angle, preserving
// continuity across +/-180 degrees rather than commanding an entire turn.
inline float wrapNear(float angle, float reference) {
  if (!finiteNumber(angle) || !finiteNumber(reference)) return NAN;
  const float delta = wrap180(wrap180(angle) - wrap180(reference));
  return reference + delta;
}

inline Point forward(const Pose &pose) {
  const float a = wrap180(pose.a) * DEGREES_TO_RADIANS;
  const float b = wrap180(pose.b) * DEGREES_TO_RADIANS;
  return Point{LINK_1_MM * cosf(a) + LINK_2_MM * cosf(b),
               LINK_1_MM * sinf(a) + LINK_2_MM * sinf(b)};
}

inline bool valid(const Pose &pose) {
  if (!finiteNumber(pose.a) || !finiteNumber(pose.b)) return false;
  const float bend = wrap180(wrap180(pose.b) - wrap180(pose.a));
  // Exclude straight and fully folded configurations. The tiny tolerance
  // permits exactly 10 degrees with single-precision AVR arithmetic.
  return fabsf(sinf(bend * DEGREES_TO_RADIANS)) >= MIN_BEND_SIN - 0.000001f;
}

// Resolve the same elbow branch as reference, with each absolute orientation
// unwrapped near reference. On failure result is left untouched.
inline bool inverse(const Point &target, const Pose &reference, Pose &result) {
  if (!finiteNumber(target.forward) || !finiteNumber(target.up) ||
      !valid(reference)) return false;

  const float radiusSquared = target.forward * target.forward +
                              target.up * target.up;
  const float maxRadius = LINK_1_MM + LINK_2_MM;
  const float minRadius = fabsf(LINK_1_MM - LINK_2_MM);
  if (!finiteNumber(radiusSquared) || radiusSquared > maxRadius * maxRadius ||
      radiusSquared <= minRadius * minRadius) return false;

  float cosine = (radiusSquared - LINK_1_MM * LINK_1_MM -
                  LINK_2_MM * LINK_2_MM) / (2.0f * LINK_1_MM * LINK_2_MM);
  if (cosine < -1.0f || cosine > 1.0f) return false;

  const float referenceBend =
      wrap180(wrap180(reference.b) - wrap180(reference.a)) * DEGREES_TO_RADIANS;
  const float branch = sinf(referenceBend) < 0.0f ? -1.0f : 1.0f;
  const float bend = branch * acosf(cosine);
  const float a = atan2f(target.up, target.forward) -
                 atan2f(LINK_2_MM * sinf(bend),
                        LINK_1_MM + LINK_2_MM * cosf(bend));
  const Pose candidate = {
      wrapNear(a * RADIANS_TO_DEGREES, reference.a),
      wrapNear((a + bend) * RADIANS_TO_DEGREES, reference.b)};
  if (!valid(candidate)) return false;
  result = candidate;
  return true;
}

// Validate the complete Cartesian segment before any motor pulse is sent.
// Checking the closest point analytically protects the inner singularity
// region even when sampling would step over it. The maximum radius lies at
// an endpoint; inverse() checks the final endpoint and valid() the start.
// The finite sample cap prevents a caller-supplied tiny spacing from blocking
// an AVR indefinitely. At 0.5 mm, every possible workspace chord fits it.
inline bool straightPathValid(const Pose &start, const Point &target,
                              float maxSegmentMM = 0.5f) {
  if (!valid(start) || !finiteNumber(maxSegmentMM) || maxSegmentMM <= 0.0f)
    return false;
  Pose finalPose;
  if (!inverse(target, start, finalPose)) return false;

  const Point origin = forward(start);
  const float dx = target.forward - origin.forward;
  const float dy = target.up - origin.up;
  const float distanceSquared = dx * dx + dy * dy;
  if (!finiteNumber(distanceSquared)) return false;
  if (distanceSquared == 0.0f) return true;

  float closestFraction = -(origin.forward * dx + origin.up * dy) /
                          distanceSquared;
  if (closestFraction < 0.0f) closestFraction = 0.0f;
  if (closestFraction > 1.0f) closestFraction = 1.0f;
  const Point closest = {origin.forward + closestFraction * dx,
                         origin.up + closestFraction * dy};
  Pose closestPose;
  if (!inverse(closest, start, closestPose)) return false;

  const float segmentCount = ceilf(sqrtf(distanceSquared) / maxSegmentMM);
  if (!finiteNumber(segmentCount) || segmentCount > 4096.0f) return false;
  const unsigned int segments = static_cast<unsigned int>(segmentCount);
  Pose previous = start;
  for (unsigned int i = 1; i <= segments; ++i) {
    const float fraction = static_cast<float>(i) / segments;
    const Point sample = {origin.forward + fraction * dx,
                          origin.up + fraction * dy};
    Pose next;
    if (!inverse(sample, previous, next)) return false;
    previous = next;
  }
  return true;
}

}  // namespace XYMotion

#endif
