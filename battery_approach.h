#ifndef ROBOT_ARM_BATTERY_APPROACH_H
#define ROBOT_ARM_BATTERY_APPROACH_H

// Recorded encoder-derived poses, not universal mechanical coordinates.
const XYMotion::Pose BATTERY_ABOVE = {28.718f, -54.385f};
const XYMotion::Pose BATTERY_PICK = {14.619f, -56.309f};
float batteryAboveZ = 31.097f;
// The original recording did not include proof of homez. Explicit confirmation
// or a freshly measured home-relative angle is required after each reset.
bool batteryZConfirmed = false;

float batteryZAngle()
{
  return zEncoderCount * (float)Z_ENC_SIGN * 360.0f / AS5600_CPR / GEAR_RATIO;
}

bool batteryLegValid(const XYMotion::Pose &start, const XYMotion::Point &target)
{
  if (!XYMotion::straightPathValid(start, target, XY_SEGMENT_MM)) return false;
  const XYMotion::Point origin = XYMotion::forward(start);
  const int segments = (int)ceil(hypot(target.forward-origin.forward, target.up-origin.up) / XY_SEGMENT_MM);
  XYMotion::Pose previous = start;
  for (int i=1; i<=segments; ++i) {
    const float f = (float)i / segments;
    const XYMotion::Point point = {origin.forward+(target.forward-origin.forward)*f,
                                  origin.up+(target.up-origin.up)*f};
    XYMotion::Pose next;
    if (!XYMotion::inverse(point, previous, next) || !xyWithinWindow(next, start)) return false;
    previous = next;
  }
  return true;
}

bool batteryMoveTo(const XYMotion::Point &target)
{
  // Short internal legs preserve the manual 50mm cap and per-move limits.
  for (int i=0; i<20; ++i) {
    if (xyStopRequested() || !xyCalibrated) return false;
    const XYMotion::Point p = XYMotion::forward(xyMeasuredPose());
    const float dx = target.forward-p.forward, dy = target.up-p.up;
    const float distance = hypot(dx,dy);
    if (distance <= XY_ENDPOINT_TOLERANCE_MM) return true;
    const float scale = distance > 10 ? 10 / distance : 1;
    xyMove(dx*scale, dy*scale, false);
    // Manual jogging keeps its reference on warnings; an automatic sequence
    // must not advance to rotation/approach after an unverified leg.
    if (!xyLastMoveVerified) return false;
  }
  return false;
}

bool batteryRotate()
{
  updateZEncoder();
  if (!zEncoderOK || !zHomed) return false;
  const float delta = batteryAboveZ-batteryZAngle();
  if (!isfinite(delta) || fabs(delta)>45) return false;
  digitalWrite(Z_PUL, LOW);
  digitalWrite(Z_DIR, (delta>=0) != Z_DIR_INVERT ? HIGH : LOW);
  if (!xyWaitMicros(20000UL)) return false;
  const long pulses = lround(fabs(delta)*PULSE_PER_DEGREE);
  for (long i=0; i<pulses; ++i) {
    if (xyStopRequested() || (delta<0 && digitalRead(Z_HOME)==HIGH)) return false;
    digitalWrite(Z_PUL,HIGH);
    const bool keepMoving = xyWaitMicros(4000UL);
    digitalWrite(Z_PUL,LOW);
    if (!keepMoving || !xyWaitMicros(4000UL)) return false;
    if (i%10==0) { updateZEncoder(); if (!zEncoderOK) return false; }
  }
  if (!xyWaitMicros(200000UL)) return false;
  updateZEncoder();
  return zEncoderOK && fabs(batteryZAngle()-batteryAboveZ)<=0.25f;
}

void batteryAbove()
{
  updateZEncoder();
  const XYMotion::Pose start = xyMeasuredPose();
  if (!xyCalibrated || !zHomed || !zEncoderOK || !batteryZConfirmed
      || xySignX!=-1 || xySignY!=1 || fabs(start.a-5)>1 || fabs(start.b+90)>1
      || fabs(batteryZAngle())>0.5f) {
    Serial.println(F("ABOVE: no motion; requires photo start pose, xycal 5 -90 -1 1, homez, abovez confirmed"));
    return;
  }
  const XYMotion::Point target = XYMotion::forward(BATTERY_ABOVE);
  const XYMotion::Point lift = {XYMotion::forward(start).forward, target.up};
  XYMotion::Pose lifted;
  if (!batteryLegValid(start,lift) || !XYMotion::inverse(lift,start,lifted)
      || !batteryLegValid(lifted,target)) {
    Serial.println(F("ABOVE: no motion; path rejected")); return;
  }
  xyOperatorStopped = false;
  if (xyStopRequested()) { Serial.println(F("ABOVE: cancelled before motion")); return; }
  gripper.write(OPEN_ANGLE);
  Serial.println(F("ABOVE: lift, rotate, approach; ! stops; no descent or close"));
  if (!xyWaitMicros(500000UL) || !batteryMoveTo(lift) || !batteryRotate() || !batteryMoveTo(target)) {
    digitalWrite(X_PUL,LOW); digitalWrite(Y_PUL,LOW); digitalWrite(Z_PUL,LOW);
    Serial.println(F("ABOVE PAUSED: sequence ended; inspect pos/e, no automatic retry")); return;
  }
  Serial.println(F("ABOVE DONE: verify battery alignment before descending"));
  xyPrintPosition();
}

bool batteryHandleCommand(const String &input)
{
  if (input == "startpose") {
    xyHandleCommand(String("xycal 5 -90 -1 1"));
    Serial.println(F("STARTPOSE: manual photo pose registered; no motion")); return true;
  }
  if (input == "above") { batteryAbove(); return true; }
  if (input == "pickinfo") {
    Serial.println(F("ABOVE: A 28.718 B -54.385; forward 175.119 up -39.895 mm"));
    Serial.println(F("PICK: A 14.619 B -56.309; forward 182.681 up -69.559 mm"));
    Serial.print(F("Saved Z DEG: ")); Serial.println(batteryAboveZ);
    Serial.println(F("abovez DEG: confirm taught home-relative Z; not a motor command")); return true;
  }
  if (input.startsWith("abovez ")) {
    float value;
    if (!xyParseNumbers(input,&value,1) || value<0 || value>45) {
      Serial.println(F("ERROR: abovez requires confirmed home-relative angle 0..45 DEG")); return true;
    }
    batteryAboveZ=value; batteryZConfirmed=true;
    Serial.println(F("ABOVE Z registered; no motion; cleared on reset")); return true;
  }
  if (input.startsWith("above") || input.startsWith("startpose") || input.startsWith("pickinfo")) {
    Serial.println(F("ERROR: use startpose, abovez DEG, above, pickinfo")); return true;
  }
  return false;
}
#endif
