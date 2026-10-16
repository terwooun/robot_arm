// Host-side software integration tests. These mocks do not establish Arduino
// compilation, electrical timing, encoder wiring, or physical motion accuracy.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class String {
 public:
  String(const char *value) : text_(value) {}
  explicit String(const std::string &value) : text_(value) {}
  const char *c_str() const { return text_.c_str(); }
  unsigned int length() const { return static_cast<unsigned int>(text_.size()); }
  char charAt(unsigned int index) const { return text_.at(index); }
  String substring(unsigned int begin, unsigned int end) const {
    return String(text_.substr(begin, end - begin));
  }
  bool startsWith(const char *prefix) const { return text_.find(prefix) == 0; }
  bool operator==(const char *other) const { return text_ == other; }
 private:
  std::string text_;
};

class MockSerial {
 public:
  std::string input;
  std::ostringstream output;
  int available() const { return static_cast<int>(input.size()); }
  int read() {
    if (input.empty()) return -1;
    const unsigned char result = input.front();
    input.erase(0, 1);
    return result;
  }
  template <typename T> void print(const T &value) { output << value; }
  void print(float value, int digits) {
    output << std::fixed << std::setprecision(digits) << value;
  }
  template <typename T> void println(const T &value) {
    print(value);
    output << '\n';
  }
  void println() { output << '\n'; }
  void clearOutput() { output.str(""); output.clear(); }
};

#define F(value) value
#define HIGH 1
#define LOW 0
// Match the preprocessor environment created by Arduino.h.
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

const int X_PUL = 6, X_DIR = 7, Y_PUL = 4, Y_DIR = 5;
const bool X_DIR_INVERT = false, Y_DIR_INVERT = false;
const int X_ENC_SIGN = -1, Y_ENC_SIGN = -1;
const double PULSE_PER_DEGREE = 3200.0 * (32.0 / 9.0) / 360.0;
const double ENC_COUNT_PER_DEGREE = 4000.0 * (32.0 / 9.0) / 360.0;
const unsigned int START_DELAY_US = 1500;
volatile long xEncoderCount = 0, yEncoderCount = 0;
MockSerial Serial;

struct Edge {
  int pin;
  int direction;
  unsigned long time;
};
static int pins[32] = {};
static unsigned long clockUs = 0;
static std::vector<Edge> edges;
static double simulatedXCount = 0, simulatedYCount = 0;
static int feedbackMultiplier = 1;
static int stopAfterRisingEdges = 0;
static int zPolls = 0;
static bool overlappedPulse = false;

unsigned long micros() { clockUs += 100; return clockUs; }
void noInterrupts() {}
void interrupts() {}
void updateZEncoder() { ++zPolls; }
void digitalWrite(int pin, int value) {
  const bool rising = value == HIGH && pins[pin] == LOW;
  pins[pin] = value;
  if (!rising || (pin != X_PUL && pin != Y_PUL)) return;
  const bool xAxis = pin == X_PUL;
  const int directionPin = xAxis ? X_DIR : Y_DIR;
  const int sign = pins[directionPin] == HIGH ? 1 : -1;
  edges.push_back(Edge{pin, sign, clockUs});
  if (pins[X_PUL] == HIGH && pins[Y_PUL] == HIGH) overlappedPulse = true;

  // A 4000-CPR encoder produces 1.25 count transitions per 3200-PPR
  // motor command pulse. Preserve fractional motion between integer reads.
  double &position = xAxis ? simulatedXCount : simulatedYCount;
  position += sign * feedbackMultiplier * -1.25;
  if (xAxis) xEncoderCount = std::lround(position);
  else yEncoderCount = std::lround(position);
  if (stopAfterRisingEdges > 0 &&
      edges.size() >= static_cast<std::size_t>(stopAfterRisingEdges)) {
    Serial.input += "!\n";
    stopAfterRisingEdges = 0;
  }
}

#include "../xy_control.h"

static void reset() {
  Serial.input.clear();
  Serial.clearOutput();
  edges.clear();
  for (int &pin : pins) pin = LOW;
  clockUs = 0;
  simulatedXCount = simulatedYCount = 0;
  xEncoderCount = yEncoderCount = 0;
  feedbackMultiplier = 1;
  stopAfterRisingEdges = 0;
  zPolls = 0;
  overlappedPulse = false;
  xyCalibrated = false;
  xyReference = XYMotion::Pose{0, 0};
  xyReferenceX = xyReferenceY = 0;
  xySignX = xySignY = 0;
}

static void command(const char *text) {
  Serial.clearOutput();
  assert(xyHandleCommand(String(text)));
}

static bool printed(const char *text) {
  return Serial.output.str().find(text) != std::string::npos;
}

static void calibrate(const char *text = "xycal 60 -30 1 1") {
  command(text);
  assert(xyCalibrated);
  assert(printed("XY CALIBRATED"));
  assert(edges.empty());
}

static long signedPulses(int pin) {
  long count = 0;
  for (const Edge &edge : edges) if (edge.pin == pin) count += edge.direction;
  return count;
}

static void expectIdle() {
  assert(pins[X_PUL] == LOW);
  assert(pins[Y_PUL] == LOW);
}

static void expectNoMotion(const char *text) {
  const std::size_t oldSize = edges.size();
  const long oldX = xEncoderCount, oldY = yEncoderCount;
  command(text);
  assert(edges.size() == oldSize);
  assert(xEncoderCount == oldX && yEncoderCount == oldY);
  expectIdle();
}

static void parserAndCalibration() {
  reset();
  for (const char *text : {"xm 1", "ym -1", "xy 1 1", "xycheck 1 1"}) {
    expectNoMotion(text);
    assert(printed("NOT CALIBRATED"));
  }
  expectNoMotion("pos");
  assert(printed("NOT CALIBRATED"));
  for (const char *text : {"xycal", "xycal 60 -30 1", "xycal 60 -30 0 1",
                           "xycal 60 -30 1 2", "xycal nan -30 1 1",
                           "xycal 361 -30 1 1", "xycal 0 0 1 1",
                           "xycal 0 180 1 1", "xycal 0 9 1 1"}) {
    expectNoMotion(text);
    assert(!xyCalibrated);
    assert(printed("ERROR"));
  }
  // Calibration snapshots existing counts and does not drive or zero motors.
  simulatedXCount = xEncoderCount = 731;
  simulatedYCount = yEncoderCount = -359;
  calibrate();
  assert(xyReferenceX == 731 && xyReferenceY == -359);
  assert(xEncoderCount == 731 && yEncoderCount == -359);
  assert(xyMeasuredPose().a == 60 && xyMeasuredPose().b == -30);
  expectNoMotion("xycal 0 0 1 1");
  assert(xyCalibrated && xyReference.a == 60 && xyReference.b == -30);
  assert(xyReferenceX == 731 && xyReferenceY == -359);
  for (const char *text : {"xm", "ym", "xy 1", "xm abc", "xm nan", "xm inf",
                           "xm 1e309", "xm 1e39", "xm 999999999999999999999",
                           "xm 1junk", "xm 1 2", "xy 1 2 3", "xy 1 NaN",
                           "xycalibration 1", "xymm 1", "ymjunk 1",
                           "xm 0", "xy 0 0", "xm 10.01", "xy 8 8"}) {
    expectNoMotion(text);
    assert(printed("ERROR"));
  }
  assert(!xyHandleCommand(String("x 5")));
  assert(!xyHandleCommand(String("y 5")));
  assert(!xyHandleCommand(String("z 5")));
  assert(!xyHandleCommand(String("h")));
}

static void previewDoesNotMutatePosition() {
  reset();
  calibrate();
  command("pos");
  const std::string before = Serial.output.str();
  expectNoMotion("xycheck 3 2");
  assert(printed("XY PREVIEW"));
  assert(printed("X COMMAND PULSE: 49"));
  assert(printed("Y COMMAND PULSE: -25"));
  assert(xyCalibrated && xyReference.a == 60 && xyReference.b == -30);
  assert(xyReferenceX == 0 && xyReferenceY == 0);
  command("pos");
  assert(Serial.output.str() == before);
}

static void expectEndpoint(float deltaForward, float deltaUp, int signX, int signY) {
  // Independent forward projection from the emulated physical encoder, rather
  // than from the planner's target or inverse-kinematics implementation.
  const double rad = std::acos(-1.0) / 180.0;
  const double a = (60 + yEncoderCount * -signY / ENC_COUNT_PER_DEGREE) * rad;
  const double b = (-30 + xEncoderCount * -signX / ENC_COUNT_PER_DEGREE) * rad;
  const double forward = 120 * std::cos(a) + 120 * std::cos(b);
  const double up = 120 * std::sin(a) + 120 * std::sin(b);
  const double originForward = 120 * std::cos(60 * rad) + 120 * std::cos(-30 * rad);
  const double originUp = 120 * std::sin(60 * rad) + 120 * std::sin(-30 * rad);
  assert(std::fabs(forward - originForward - deltaForward) < 0.12);
  assert(std::fabs(up - originUp - deltaUp) < 0.12);
}

static void synchronizedMoves() {
  struct Move { const char *text; float forward, up; long x, y; };
  // Expected endpoint counts are fixed fixtures for two 120 mm links at
  // A=60, B=-30 degrees and 3200 pulses/revolution with the 32:9 reduction.
  const Move moves[] = {{"xm 3", 3, 0, 23, -39}, {"ym 3", 0, 3, 39, 22},
                        {"xy 3 2", 3, 2, 49, -25}, {"xy 2 -1", 2, -1, 2, -34}};
  for (const Move &move : moves) {
    reset();
    calibrate();
    command(move.text);
    assert(printed("XY DONE"));
    assert(!printed("ERROR: XY"));
    assert(signedPulses(X_PUL) == move.x);
    assert(signedPulses(Y_PUL) == move.y);
    assert(zPolls > 0);
    assert(overlappedPulse);
    assert(xyCalibrated);
    expectEndpoint(move.forward, move.up, 1, 1);
    expectIdle();
  }
  for (int signX : {-1, 1}) {
    for (int signY : {-1, 1}) {
      reset();
      std::ostringstream calibration;
      calibration << "xycal 60 -30 " << signX << ' ' << signY;
      calibrate(calibration.str().c_str());
      command("xy 3 2");
      assert(printed("XY DONE"));
      assert(signedPulses(X_PUL) == 49 * signX);
      assert(signedPulses(Y_PUL) == -25 * signY);
      expectEndpoint(3, 2, signX, signY);
      expectIdle();
    }
  }
}

static void rejectsWholePathBeforePulsing() {
  reset();
  calibrate("xycal 0 12 1 1");
  expectNoMotion("xm -5");
  assert(printed("exceeds joint test window"));
  expectNoMotion("xm 10");
  assert(printed("unreachable"));

  reset();
  calibrate("xycal 0 11 1 1");
  expectNoMotion("xm 1");
  assert(printed("singularity"));

  reset();
  calibrate();
  // An external manual movement outside the calibration window is detected
  // from measured encoder position before the next command starts.
  simulatedYCount = yEncoderCount = std::lround(-16 * ENC_COUNT_PER_DEGREE);
  expectNoMotion("xm 1");
  assert(printed("outside calibration window"));
}

static void stopAndFollowingErrors() {
  reset();
  calibrate();
  stopAfterRisingEdges = 1;
  command("xy 3 2");
  assert(!edges.empty() && edges.size() <= 2);
  assert(printed("XY ABORTED"));
  assert(!xyCalibrated);
  expectIdle();
  expectNoMotion("xm 1");
  assert(printed("NOT CALIBRATED"));

  for (const int feedback : {0, -1}) {
    reset();
    calibrate();
    feedbackMultiplier = feedback;
    command("xm 10");
    assert(!edges.empty());
    assert(printed("ENCODER FOLLOWING ERROR"));
    assert(printed("XY ABORTED"));
    assert(!printed("XY DONE"));
    assert(!xyCalibrated);
    assert(std::labs(signedPulses(Y_PUL)) < 133);
    expectIdle();
  }

  reset();
  calibrate();
  xyInvalidateReference();  // The sketch calls this when legacy zero is used.
  expectNoMotion("ym 1");
  assert(printed("NOT CALIBRATED"));
  reset();
  calibrate();
  expectNoMotion("!");
  assert(!xyCalibrated);
}

int main() {
  parserAndCalibration();
  previewDoesNotMutatePosition();
  synchronizedMoves();
  rejectsWholePathBeforePulsing();
  stopAndFollowingErrors();
  std::cout << "XY control mock integration tests passed\n";
}
