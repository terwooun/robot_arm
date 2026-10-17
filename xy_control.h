#ifndef ROBOT_ARM_XY_CONTROL_H
#define ROBOT_ARM_XY_CONTROL_H

#include "xy_motion.h"
#include <stdlib.h>
#include <ctype.h>

// Relative motion in the vertical plane of the arm; Z rotates this plane.
// Physical link vectors: a = base -> elbow (Y), b = elbow -> wrist (X).
// These are absolute angles from the forward horizontal, positive toward up.
// A fixed gripper offset cancels in relative moves while its orientation is held.
const float XY_MAX_MOVE_MM = 50.0f;
const float XY_SEGMENT_MM = 0.5f;
const float XY_MAX_COMMAND_DEG = 45.0f;
const float XY_CAL_WINDOW_DEG = 60.0f;
const float XY_MAX_ERROR_DEG = 1.0f;
const float XY_START_ERROR_DEG = 0.15f;
// Only small residuals may enter endpoint correction; larger errors still abort.
const float XY_FINAL_ERROR_DEG = 0.25f;
const float XY_ENDPOINT_TOLERANCE_MM = 0.15f;
const int XY_CORRECTION_ATTEMPTS = 3;
const long XY_CORRECTION_STEP_PULSES = 6;
const long XY_CORRECTION_TOTAL_PULSES = 15; // Per axis, total absolute extra travel.
const float XY_FEED_MM_S = 2.0f;
const float XY_ACCEL_MM_S2 = 2.0f;

bool xyCalibrated = false;
bool xyOperatorStopped = false;
XYMotion::Pose xyReference = {0.0f, 0.0f};
long xyReferenceX = 0;
long xyReferenceY = 0;
int xySignX = 0;
int xySignY = 0;

void updateZEncoder();

void xyReadCounts(long &x, long &y)
{
  noInterrupts();
  x = xEncoderCount;
  y = yEncoderCount;
  interrupts();
}

XYMotion::Pose xyPoseFromCounts(long x, long y)
{
  XYMotion::Pose pose;
  pose.a = xyReference.a + ((float)y - xyReferenceY) * Y_ENC_SIGN * xySignY / ENC_COUNT_PER_DEGREE;
  pose.b = xyReference.b + ((float)x - xyReferenceX) * X_ENC_SIGN * xySignX / ENC_COUNT_PER_DEGREE;
  return pose;
}

XYMotion::Pose xyMeasuredPose()
{
  long x, y;
  xyReadCounts(x, y);
  return xyPoseFromCounts(x, y);
}

void xyInvalidateReference()
{
  if (xyCalibrated)
    Serial.println(F("XY REFERENCE CLEARED: register measured pose with xycal again"));
  xyCalibrated = false;
}

// Strict finite decimal parsing: malformed XY commands must not reach x/y.
bool xyParseNumbers(const String &input, float *values, uint8_t count)
{
  const char *p = input.c_str();
  while (*p && !isspace((unsigned char)*p)) ++p;
  for (uint8_t i = 0; i < count; ++i)
  {
    while (isspace((unsigned char)*p)) ++p;
    if (!*p) return false;
    char *end;
    double value = strtod(p, &end);
    if (end == p || !isfinite(value) || (*end && !isspace((unsigned char)*end))) return false;
    values[i] = (float)value;
    if (!isfinite(values[i])) return false;
    p = end;
  }
  while (isspace((unsigned char)*p)) ++p;
  return *p == '\0';
}

void xyHelp()
{
  Serial.println(F("XY mm: xycal A B SX SY (measured link angles DEG; direction signs +/-1)"));
  Serial.println(F("A=base->elbow (Y), B=elbow->wrist (X); forward=0, up=+90"));
  Serial.println(F("SX/SY: +1 if positive motor command increases its link angle, else -1"));
  Serial.println(F("xm 1 = forward 1 mm; ym 1 = up 1 mm; xy 1 1 = diagonal"));
  Serial.println(F("xycheck 1 0 = preview only; pos = encoder-derived wrist position"));
  Serial.println(F("! + Send = stop during mm motion; other input during mm motion is discarded"));
  Serial.println(F("Initial test limits: 50 mm/move, 45 DEG/move, +/-60 DEG from calibration"));
  Serial.println(F("h = switch inputs; homez = Z homing; X/Y homing not implemented"));
}

void xyPrintPosition()
{
  if (!xyCalibrated)
  {
    Serial.println(F("ERROR: XY NOT CALIBRATED. Use xyhelp and xycal"));
    return;
  }
  XYMotion::Pose pose = xyMeasuredPose();
  XYMotion::Point point = XYMotion::forward(pose);
  Serial.print(F("XY LINK A/Y: ")); Serial.print(pose.a, 3);
  Serial.print(F(" DEG, B/X: ")); Serial.print(pose.b, 3); Serial.println(F(" DEG"));
  Serial.print(F("WRIST relative to base joint: FORWARD ")); Serial.print(point.forward, 3);
  Serial.print(F(" mm, UP ")); Serial.print(point.up, 3); Serial.println(F(" mm"));
}

bool xyWithinWindow(const XYMotion::Pose &pose, const XYMotion::Pose &start)
{
  return XYMotion::valid(pose)
    && fabs(pose.a - start.a) <= XY_MAX_COMMAND_DEG
    && fabs(pose.b - start.b) <= XY_MAX_COMMAND_DEG
    && fabs(pose.a - xyReference.a) <= XY_CAL_WINDOW_DEG
    && fabs(pose.b - xyReference.b) <= XY_CAL_WINDOW_DEG;
}

bool xyStopRequested()
{
  bool stopped = false;
  while (Serial.available() > 0)
  {
    if (Serial.read() == '!') stopped = true;
  }
  if (stopped) xyOperatorStopped = true;
  return stopped;
}

bool xyWaitMicros(unsigned long duration)
{
  const unsigned long began = micros();
  do
  {
    if (xyStopRequested()) return false;
  } while ((unsigned long)(micros() - began) < duration);
  return true;
}

// DDA distributes both pulse trains over the same time interval.
bool xyPulsePair(long stepsX, long stepsY, unsigned long durationUs)
{
  const long countX = labs(stepsX), countY = labs(stepsY);
  const long ticks = countX > countY ? countX : countY;
  if (ticks == 0) return xyWaitMicros(durationUs);
  if (xyStopRequested()) return false;
  if (countX) digitalWrite(X_DIR, (stepsX > 0) != X_DIR_INVERT ? HIGH : LOW);
  if (countY) digitalWrite(Y_DIR, (stepsY > 0) != Y_DIR_INVERT ? HIGH : LOW);
  // Driver direction setup; no extra pulse is emitted while switching direction.
  if (!xyWaitMicros(20UL)) return false;
  unsigned long halfPeriod = durationUs / (2UL * (unsigned long)ticks);
  if (halfPeriod < START_DELAY_US) halfPeriod = START_DELAY_US;
  long errorX = ticks / 2, errorY = ticks / 2;
  for (long i = 0; i < ticks; ++i)
  {
    if (xyStopRequested()) return false;
    errorX += countX; errorY += countY;
    const bool pulseX = errorX >= ticks;
    const bool pulseY = errorY >= ticks;
    if (pulseX) { errorX -= ticks; digitalWrite(X_PUL, HIGH); }
    if (pulseY) { errorY -= ticks; digitalWrite(Y_PUL, HIGH); }
    const bool keepMoving = xyWaitMicros(halfPeriod);
    digitalWrite(X_PUL, LOW);
    digitalWrite(Y_PUL, LOW);
    if (!keepMoving || !xyWaitMicros(halfPeriod)) return false;
  }
  return true;
}

bool xyFeedbackOK(long startX, long startY, long sentX, long sentY, float tolerance = XY_MAX_ERROR_DEG)
{
  long x, y;
  xyReadCounts(x, y);
  const float actualX = ((float)x - startX) * X_ENC_SIGN / ENC_COUNT_PER_DEGREE;
  const float actualY = ((float)y - startY) * Y_ENC_SIGN / ENC_COUNT_PER_DEGREE;
  const float errorX = actualX - sentX / PULSE_PER_DEGREE;
  const float errorY = actualY - sentY / PULSE_PER_DEGREE;
  if (fabs(errorX) <= tolerance && fabs(errorY) <= tolerance) return true;
  Serial.print(F("WARNING: XY ENCODER FOLLOWING ERROR; X ")); Serial.print(errorX, 3);
  Serial.print(F(" DEG, Y ")); Serial.print(errorY, 3); Serial.println(F(" DEG"));
  return false;
}

void xyAbort()
{
  digitalWrite(X_PUL, LOW);
  digitalWrite(Y_PUL, LOW);
  Serial.println(F("XY ABORTED: pulses stopped; motors remain enabled"));
  xyInvalidateReference();
}

float xyEndpointError(const XYMotion::Pose &pose, const XYMotion::Point &target)
{
  const XYMotion::Point actual = XYMotion::forward(pose);
  return hypot(actual.forward - target.forward, actual.up - target.up);
}

// Bounded low-speed endpoint correction, not continuous servo control.
// Never change the target to the current position or restart the main move.
bool xyCorrectEndpoint(const XYMotion::Pose &goal, const XYMotion::Point &target,
                       const XYMotion::Pose &start)
{
  long totalX = 0, totalY = 0;
  for (int attempt = 0; attempt <= XY_CORRECTION_ATTEMPTS; ++attempt)
  {
    if (xyStopRequested()) return false;
    long beforeX, beforeY;
    xyReadCounts(beforeX, beforeY);
    const XYMotion::Pose current = xyPoseFromCounts(beforeX, beforeY);
    const float error = xyEndpointError(current, target);
    if (!isfinite(error) || !xyWithinWindow(current, start)
        || fabs(goal.a - current.a) > XY_FINAL_ERROR_DEG
        || fabs(goal.b - current.b) > XY_FINAL_ERROR_DEG)
    {
      Serial.println(F("WARNING: XY correction outside small-error window"));
      return false;
    }
    if (error <= XY_ENDPOINT_TOLERANCE_MM) return true;
    if (attempt == XY_CORRECTION_ATTEMPTS) break;
    long stepsX = lround((goal.b - current.b) * xySignX * PULSE_PER_DEGREE);
    long stepsY = lround((goal.a - current.a) * xySignY * PULSE_PER_DEGREE);
    const long largest = labs(stepsX) > labs(stepsY) ? labs(stepsX) : labs(stepsY);
    if (largest > XY_CORRECTION_STEP_PULSES)
    {
      stepsX = lround((float)stepsX * XY_CORRECTION_STEP_PULSES / largest);
      stepsY = lround((float)stepsY * XY_CORRECTION_STEP_PULSES / largest);
    }
    if ((stepsX == 0 && stepsY == 0)
        || totalX + labs(stepsX) > XY_CORRECTION_TOTAL_PULSES
        || totalY + labs(stepsY) > XY_CORRECTION_TOTAL_PULSES)
    {
      Serial.println(F("WARNING: XY correction pulse budget/resolution limit"));
      return false;
    }
    const XYMotion::Pose predicted = {
      current.a + stepsY * xySignY / (float)PULSE_PER_DEGREE,
      current.b + stepsX * xySignX / (float)PULSE_PER_DEGREE};
    if (!xyWithinWindow(predicted, start)
        || !XYMotion::straightPathValid(current, target, XY_SEGMENT_MM))
    {
      Serial.println(F("WARNING: XY correction path rejected"));
      return false;
    }
    Serial.print(F("XY CORRECT ")); Serial.print(attempt + 1);
    Serial.print(F(": error ")); Serial.print(error, 3);
    Serial.print(F(" mm; X pulses ")); Serial.print(stepsX);
    Serial.print(F(", Y pulses ")); Serial.println(stepsY);
    const long ticks = labs(stepsX) > labs(stepsY) ? labs(stepsX) : labs(stepsY);
    if (!xyPulsePair(stepsX, stepsY, (unsigned long)ticks * 12000UL)) return false;
    totalX += labs(stepsX); totalY += labs(stepsY);
    if (!xyWaitMicros(200000UL)
        || !xyFeedbackOK(beforeX, beforeY, stepsX, stepsY, XY_FINAL_ERROR_DEG)) return false;
    updateZEncoder();
    const float afterError = xyEndpointError(xyMeasuredPose(), target);
    if (!isfinite(afterError)
        || (afterError > XY_ENDPOINT_TOLERANCE_MM && afterError > error - 0.02f))
    {
      Serial.println(F("WARNING: XY correction not improving; stopped"));
      return false;
    }
  }
  Serial.println(F("WARNING: XY correction attempts exhausted"));
  return false;
}

void xyMove(float deltaForward, float deltaUp, bool preview)
{
  if (!xyCalibrated)
  {
    Serial.println(F("ERROR: XY NOT CALIBRATED. Use xyhelp and xycal"));
    return;
  }
  const float distance = sqrt(deltaForward * deltaForward + deltaUp * deltaUp);
  if (!isfinite(distance) || distance <= 0.0f || distance > XY_MAX_MOVE_MM)
  {
    Serial.println(F("ERROR: XY distance must be >0 and <=50 mm"));
    return;
  }
  long startX, startY;
  xyReadCounts(startX, startY);
  const XYMotion::Pose start = xyPoseFromCounts(startX, startY);
  const XYMotion::Point origin = XYMotion::forward(start);
  const XYMotion::Point target = {origin.forward + deltaForward, origin.up + deltaUp};
  if (!xyWithinWindow(start, start) || !XYMotion::straightPathValid(start, target, XY_SEGMENT_MM))
  {
    Serial.println(F("ERROR: XY path unreachable, near singularity or outside calibration window"));
    return;
  }
  const int segments = (int)ceil(distance / XY_SEGMENT_MM);
  XYMotion::Pose previous = start;
  long finalX = 0, finalY = 0;
  // Check the entire path before emitting any pulses; use cumulative rounding.
  for (int i = 1; i <= segments; ++i)
  {
    const float fraction = (float)i / segments;
    const XYMotion::Point point = {origin.forward + deltaForward * fraction, origin.up + deltaUp * fraction};
    XYMotion::Pose next;
    if (!XYMotion::inverse(point, previous, next) || !xyWithinWindow(next, start))
    {
      Serial.println(F("ERROR: XY path exceeds joint test window; no motion"));
      return;
    }
    finalX = lround((next.b - start.b) * xySignX * PULSE_PER_DEGREE);
    finalY = lround((next.a - start.a) * xySignY * PULSE_PER_DEGREE);
    previous = next;
  }
  Serial.print(preview ? F("XY PREVIEW: ") : F("XY MOVE: "));
  Serial.print(deltaForward, 3); Serial.print(F(" mm forward, "));
  Serial.print(deltaUp, 3); Serial.println(F(" mm up"));
  Serial.print(F("X COMMAND PULSE: ")); Serial.println(finalX);
  Serial.print(F("Y COMMAND PULSE: ")); Serial.println(finalY);
  if (preview) return;
  if (finalX == 0 && finalY == 0)
  {
    Serial.println(F("XY NO MOTION: below pulse resolution"));
    return;
  }
  xyOperatorStopped = false;
  if (!xyFeedbackOK(startX, startY, 0, 0, XY_START_ERROR_DEG)) {
    Serial.println(F("WARNING: XY pose changed during planning; no motion, reference retained"));
    return;
  }
  bool followingWarning = false;
  long sentX = 0, sentY = 0;
  previous = start;
  const float segmentDistance = distance / segments;
  for (int i = 1; i <= segments; ++i)
  {
    const float fraction = (float)i / segments;
    const XYMotion::Point point = {origin.forward + deltaForward * fraction, origin.up + deltaUp * fraction};
    XYMotion::Pose next;
    if (!XYMotion::inverse(point, previous, next)) { xyAbort(); return; }
    const long targetX = lround((next.b - start.b) * xySignX * PULSE_PER_DEGREE);
    const long targetY = lround((next.a - start.a) * xySignY * PULSE_PER_DEGREE);
    // Approximate a slow acceleration/deceleration profile over the path.
    const float along = (i - 0.5f) * segmentDistance;
    const float edgeDistance = along < distance - along ? along : distance - along;
    float speed = sqrt(2.0f * XY_ACCEL_MM_S2 * edgeDistance);
    if (speed > XY_FEED_MM_S) speed = XY_FEED_MM_S;
    if (speed < 0.25f) speed = 0.25f;
    const unsigned long duration = (unsigned long)(segmentDistance / speed * 1000000.0f);
    if (!xyPulsePair(targetX - sentX, targetY - sentY, duration)) { xyAbort(); return; }
    sentX = targetX; sentY = targetY;
    if (!followingWarning) followingWarning = !xyFeedbackOK(startX, startY, sentX, sentY);
    updateZEncoder();
    previous = next;
  }
  if (!xyWaitMicros(200000UL)) { xyAbort(); return; }
  const bool smallResidual = xyFeedbackOK(startX, startY, sentX, sentY, XY_FINAL_ERROR_DEG);
  // Never chase a large error. Complete the requested pulses, retain the
  // reference, and report measured position without claiming target reached.
  bool corrected = false;
  if (smallResidual && !followingWarning) corrected = xyCorrectEndpoint(previous, target, start);
  if (xyOperatorStopped) { xyAbort(); return; }
  if (!corrected)
    Serial.println(F("WARNING: XY target not verified; correction skipped/limited; reference retained"));
  const XYMotion::Point actual = XYMotion::forward(xyMeasuredPose());
  Serial.print(corrected ? F("XY DONE (encoder-derived): FORWARD ") : F("XY FINISHED WITH WARNING (encoder-derived): FORWARD ")); Serial.print(actual.forward - origin.forward, 3);
  Serial.print(F(" mm, UP ")); Serial.print(actual.up - origin.up, 3); Serial.println(F(" mm"));
  Serial.print(F("XY ERROR: FORWARD ")); Serial.print(actual.forward - target.forward, 3);
  Serial.print(F(" mm, UP ")); Serial.print(actual.up - target.up, 3); Serial.println(F(" mm"));
}

bool xyHandleCommand(const String &input)
{
  int end = 0;
  while (end < (int)input.length() && !isspace((unsigned char)input.charAt(end))) ++end;
  const String command = input.substring(0, end);
  if (command == "xyhelp") { xyHelp(); return true; }
  if (command == "pos") { xyPrintPosition(); return true; }
  if (command == "!") { xyInvalidateReference(); Serial.println(F("XY STOP: idle")); return true; }
  if (command == "xycal")
  {
    float values[4];
    if (!xyParseNumbers(input, values, 4) || fabs(values[0]) > 360.0f || fabs(values[1]) > 360.0f
        || fabs(values[2]) != 1.0f || fabs(values[3]) != 1.0f)
    {
      Serial.println(F("ERROR: xycal A B SX SY; measured A/B degrees, SX/SY each +1 or -1"));
      return true;
    }
    const XYMotion::Pose pose = {values[0], values[1]};
    if (!XYMotion::valid(pose))
    {
      Serial.println(F("ERROR: XY calibration near straight/folded singularity"));
      return true;
    }
    xyReference = pose;
    xySignX = (int)values[2]; xySignY = (int)values[3];
    xyReadCounts(xyReferenceX, xyReferenceY);
    xyCalibrated = true;
    Serial.println(F("XY CALIBRATED: no motion; temporary test window +/-60 DEG"));
    xyPrintPosition();
    return true;
  }
  if (command == "xm" || command == "ym" || command == "xy" || command == "xycheck")
  {
    float values[2] = {0.0f, 0.0f};
    const bool pair = command == "xy" || command == "xycheck";
    if (!xyParseNumbers(input, values, pair ? 2 : 1))
    {
      Serial.println(F("ERROR: xm MM | ym MM | xy FORWARD_MM UP_MM | xycheck FORWARD_MM UP_MM"));
      return true;
    }
    if (command == "ym") { values[1] = values[0]; values[0] = 0.0f; }
    xyMove(values[0], values[1], command == "xycheck");
    return true;
  }
  // Prevent typos such as xymm/xycalibration being interpreted as legacy x motion.
  if (command.startsWith("xy") || command.startsWith("xm") || command.startsWith("ym"))
  {
    Serial.println(F("ERROR: unknown XY command. Use xyhelp"));
    return true;
  }
  return false;
}

#endif
