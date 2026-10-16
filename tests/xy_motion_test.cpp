// Arduino.h defines these as macros before sketch headers are included.
// Keep the standalone header compatible with that preprocessing environment.
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#include "../xy_motion.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using XYMotion::Point;
using XYMotion::Pose;

static bool near(float actual, float expected, float tolerance = 0.002f) {
  return std::fabs(actual - expected) <= tolerance;
}

static void expectPoint(const Point &actual, const Point &expected) {
  assert(near(actual.forward, expected.forward));
  assert(near(actual.up, expected.up));
}

static void roundTrips() {
  // Both mechanically possible branches and angles crossing +/-180 degrees.
  const float bends[] = {-169.0f, -120.0f, -60.0f, -11.0f,
                          11.0f,   60.0f, 120.0f, 169.0f};
  for (float a = -350.0f; a <= 350.0f; a += 7.0f) {
    for (float bend : bends) {
      const Pose original = {a, a + bend};
      const Point point = XYMotion::forward(original);
      Pose restored = {0.0f, 0.0f};
      assert(XYMotion::valid(original));
      assert(XYMotion::inverse(point, original, restored));
      assert(near(restored.a, original.a));
      assert(near(restored.b, original.b));
      expectPoint(XYMotion::forward(restored), point);
    }
  }
}

static void branchesAndContinuity() {
  const Point point = XYMotion::forward(Pose{60.0f, -30.0f});
  Pose negative = {0.0f, 0.0f};
  Pose positive = {0.0f, 0.0f};
  assert(XYMotion::inverse(point, Pose{60.0f, -30.0f}, negative));
  assert(XYMotion::inverse(point, Pose{-30.0f, 60.0f}, positive));
  assert(near(negative.a, 60.0f) && near(negative.b, -30.0f));
  assert(near(positive.a, -30.0f) && near(positive.b, 60.0f));

  Pose previous = {170.0f, 230.0f};
  for (int i = 1; i <= 40; ++i) {
    const Pose expected = {170.0f + i, 230.0f + i};
    Pose next;
    assert(XYMotion::inverse(XYMotion::forward(expected), previous, next));
    assert(near(next.a, expected.a) && near(next.b, expected.b));
    assert(std::fabs(next.a - previous.a) < 1.01f);
    assert(std::fabs(next.b - previous.b) < 1.01f);
    previous = next;
  }
  assert(near(XYMotion::wrapNear(-179.0f, 179.0f), 181.0f));
  assert(near(XYMotion::wrapNear(179.0f, -179.0f), -181.0f));
}

static void straightSegments() {
  for (int branch = -1; branch <= 1; branch += 2) {
    const Pose start = {branch * 60.0f, branch * -30.0f};
    const Point origin = XYMotion::forward(start);
    const Point displacement[] = {{10.0f, 0.0f}, {0.0f, 10.0f},
                                  {-10.0f, -10.0f}};
    for (const Point &offset : displacement) {
      Pose previous = start;
      for (int step = 1; step <= 100; ++step) {
        const float fraction = step / 100.0f;
        const Point target = {origin.forward + offset.forward * fraction,
                              origin.up + offset.up * fraction};
        Pose next;
        assert(XYMotion::inverse(target, previous, next));
        expectPoint(XYMotion::forward(next), target);
        assert(std::fabs(next.a - previous.a) < 1.0f);
        assert(std::fabs(next.b - previous.b) < 1.0f);
        previous = next;
      }
    }
  }
}

static void rejectionAndFailureContract() {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  const Pose reference = {60.0f, -30.0f};
  const Pose sentinel = {123.0f, 456.0f};
  const Point invalid[] = {{241.0f, 0.0f}, {0.0f, 0.0f}, {240.0f, 0.0f},
                           {239.9f, 0.0f}, {10.0f, 0.0f},
                           {nan, 50.0f}, {50.0f, nan}, {inf, 0.0f},
                           {FLT_MAX, FLT_MAX}};
  for (const Point &target : invalid) {
    Pose result = sentinel;
    assert(!XYMotion::inverse(target, reference, result));
    assert(result.a == sentinel.a && result.b == sentinel.b);
  }
  assert(!XYMotion::valid(Pose{0.0f, 0.0f}));
  assert(!XYMotion::valid(Pose{0.0f, 180.0f}));
  assert(!XYMotion::valid(Pose{0.0f, 9.9f}));
  assert(!XYMotion::valid(Pose{0.0f, 170.1f}));
  assert(XYMotion::valid(Pose{0.0f, 10.0f}));
  assert(XYMotion::valid(Pose{0.0f, 170.0f}));
  assert(!XYMotion::valid(Pose{nan, 60.0f}));
  assert(!XYMotion::valid(Pose{60.0f, inf}));
  Pose result = sentinel;
  assert(!XYMotion::inverse(Point{120.0f, 120.0f}, Pose{0.0f, 0.0f}, result));
  assert(result.a == sentinel.a && result.b == sentinel.b);

  // Valid endpoints do not imply a valid straight path: this chord crosses
  // the fully folded region. The caller must preflight every path segment.
  Pose left;
  Pose right;
  assert(XYMotion::inverse(Point{-100.0f, 0.0f}, reference, left));
  assert(XYMotion::inverse(Point{100.0f, 0.0f}, reference, right));
  assert(!XYMotion::inverse(Point{0.0f, 0.0f}, left, result));
}

static void legacyMinusAngleConvention() {
  // A linkage convention with -L2*cos(theta2), -L2*sin(theta2) is
  // equivalent to the physical elbow->wrist angle b = theta2 + 180.
  for (int theta1 = -120; theta1 <= 120; theta1 += 20) {
    for (int theta2 = -120; theta2 <= 120; theta2 += 20) {
      const float a = theta1 * XYMotion::DEGREES_TO_RADIANS;
      const float t2 = theta2 * XYMotion::DEGREES_TO_RADIANS;
      const Point expected = {
          120.0f * std::cos(a) - 120.0f * std::cos(t2),
          120.0f * std::sin(a) - 120.0f * std::sin(t2)};
      expectPoint(XYMotion::forward(Pose{float(theta1), float(theta2 + 180)}),
                  expected);
    }
  }
}

static void completePathValidation() {
  const Pose start = {60.0f, -30.0f};
  const Point origin = XYMotion::forward(start);
  assert(XYMotion::straightPathValid(start, origin));
  assert(XYMotion::straightPathValid(start, Point{origin.forward + 10.0f,
                                                 origin.up + 10.0f}));
  assert(!XYMotion::straightPathValid(start, Point{300.0f, 0.0f}));
  assert(!XYMotion::straightPathValid(start, origin, 0.0f));
  assert(!XYMotion::straightPathValid(start, origin, -1.0f));
  assert(!XYMotion::straightPathValid(start, origin,
      std::numeric_limits<float>::infinity()));

  // A 1 mm chord at y=20.915 has valid endpoints but crosses inside the
  // 10-degree folded singularity guard. Deliberately use one sample to
  // ensure the analytic closest-point check catches the invalid interior.
  Pose chordStart;
  assert(XYMotion::inverse(Point{-0.5f, 20.915f}, start, chordStart));
  Pose chordEnd;
  assert(XYMotion::inverse(Point{0.5f, 20.915f}, chordStart, chordEnd));
  assert(!XYMotion::straightPathValid(chordStart, Point{0.5f, 20.915f}, 2.0f));

  // A chord through the origin also cannot be made valid by coarse sampling.
  assert(XYMotion::inverse(Point{-100.0f, 0.0f}, start, chordStart));
  assert(!XYMotion::straightPathValid(chordStart, Point{100.0f, 0.0f}, 300.0f));
  assert(!XYMotion::straightPathValid(start,
      Point{origin.forward + 1.0f, origin.up}, 0.000001f));
}

int main() {
  roundTrips();
  branchesAndContinuity();
  straightSegments();
  rejectionAndFailureContract();
  legacyMinusAngleConvention();
  completePathValidation();
  std::cout << "XY kinematics tests passed\n";
  return 0;
}
