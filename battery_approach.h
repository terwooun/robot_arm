#ifndef ROBOT_ARM_BATTERY_APPROACH_H
#define ROBOT_ARM_BATTERY_APPROACH_H

// Recorded encoder-derived poses, not universal mechanical coordinates.
const XYMotion::Pose BATTERY_ABOVE = {29.097f, -54.765f};
const XYMotion::Pose BATTERY_PICK = {14.619f, -56.309f};
float batteryAboveZ = 28.056f;
// Home-relative Z confirmed by the latest homez + manual alignment log.
// Default is restored after reset; abovez is an optional session override.
bool batteryHadWarning = false;

float batteryZAngle()
{
  return zEncoderCount * (float)Z_ENC_SIGN * 360.0f / AS5600_CPR / GEAR_RATIO;
}

// Joint-space interpolation: all three axes share one DDA progress variable.
// The wrist follows a curved path, not the previous lift/rotate/advance route.
bool batteryPathValid(const XYMotion::Pose &start)
{
  for (int i=0; i<=200; ++i) {
    const float f = i/200.0f;
    const XYMotion::Pose p = {start.a+(BATTERY_ABOVE.a-start.a)*f,
                              start.b+(BATTERY_ABOVE.b-start.b)*f};
    if (!xyWithinWindow(p,start)) return false;
  }
  return true;
}

bool batterySynchronizedMove(const XYMotion::Pose &start)
{
  long beforeX,beforeY;
  xyReadCounts(beforeX,beforeY);
  const float dz=batteryAboveZ-batteryZAngle();
  if (!isfinite(dz) || fabs(dz)>45 || !zEncoderOK) return false;
  const long sx=lround((BATTERY_ABOVE.b-start.b)*xySignX*PULSE_PER_DEGREE);
  const long sy=lround((BATTERY_ABOVE.a-start.a)*xySignY*PULSE_PER_DEGREE);
  const long sz=lround(dz*PULSE_PER_DEGREE);
  const long nx=labs(sx), ny=labs(sy), nz=labs(sz);
  const long xyTicks=nx>ny ? nx:ny;
  const long ticks=xyTicks>nz ? xyTicks:nz;
  if (!ticks) return true;
  digitalWrite(X_PUL,LOW); digitalWrite(Y_PUL,LOW); digitalWrite(Z_PUL,LOW);
  digitalWrite(X_DIR,(sx>=0)!=X_DIR_INVERT ? HIGH:LOW);
  digitalWrite(Y_DIR,(sy>=0)!=Y_DIR_INVERT ? HIGH:LOW);
  digitalWrite(Z_DIR,(sz>=0)!=Z_DIR_INVERT ? HIGH:LOW);
  if (!xyWaitMicros(20000UL)) return false;
  long ex=ticks/2,ey=ticks/2,ez=ticks/2;
  const float da=BATTERY_ABOVE.a-start.a, db=BATTERY_ABOVE.b-start.b;
  // Integrate the XY wrist arc for a smooth acceleration/deceleration envelope.
  float length=0;
  XYMotion::Point previous=XYMotion::forward(start);
  for(int i=1;i<=200;++i) {
    const float f=i/200.0f;
    const XYMotion::Point p=XYMotion::forward(XYMotion::Pose{start.a+da*f,start.b+db*f});
    length+=hypot(p.forward-previous.forward,p.up-previous.up); previous=p;
  }
  previous=XYMotion::forward(start);
  float travelled=0;
  for(long i=0;i<ticks;++i) {
    if(xyStopRequested() || (sz<0 && digitalRead(Z_HOME)==HIGH)) return false;
    const float f=(i+1.0f)/ticks;
    const XYMotion::Point p=XYMotion::forward(XYMotion::Pose{start.a+da*f,start.b+db*f});
    const float stepDistance=hypot(p.forward-previous.forward,p.up-previous.up);
    const float along=travelled+stepDistance*0.5f;
    const float edge=fmax(0.0f,fmin(along,length-along));
    float speed=fmin(XY_FEED_MM_S,sqrt(2.0f*XY_ACCEL_MM_S2*edge));
    speed=fmax(0.25f,speed);
    unsigned long halfPeriod=(unsigned long)(stepDistance/speed*500000.0f);
    // Z never exceeds the old 4ms HIGH + 4ms LOW pulse rate.
    const unsigned long minimum=nz ? 4000UL : START_DELAY_US;
    if(halfPeriod<minimum) halfPeriod=minimum;
    ex+=nx; ey+=ny; ez+=nz;
    const bool px=ex>=ticks, py=ey>=ticks, pz=ez>=ticks;
    if(px) {ex-=ticks;digitalWrite(X_PUL,HIGH);}
    if(py) {ey-=ticks;digitalWrite(Y_PUL,HIGH);}
    if(pz) {ez-=ticks;digitalWrite(Z_PUL,HIGH);}
    const bool keepMoving=xyWaitMicros(halfPeriod);
    digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
    if(!keepMoving || !xyWaitMicros(halfPeriod)) return false;
    travelled+=stepDistance; previous=p;
    if(i%16==0) {updateZEncoder();if(!zEncoderOK) return false;}
  }
  if(!xyWaitMicros(200000UL)) return false;
  updateZEncoder();
  if(!zEncoderOK) return false;
  const bool small=xyFeedbackOK(beforeX,beforeY,sx,sy,XY_FINAL_ERROR_DEG);
  if(small) {
    if(!xyCorrectEndpoint(BATTERY_ABOVE,XYMotion::forward(BATTERY_ABOVE),start))
      batteryHadWarning=true;
    if(xyOperatorStopped) return false;
  } else batteryHadWarning=true;
  const float error=batteryZAngle()-batteryAboveZ;
  if(!isfinite(error)) return false;
  if(fabs(error)>0.25f) {
    batteryHadWarning=true;
    Serial.print(F("WARNING: ABOVE Z angle error DEG: "));Serial.println(error);
  }
  return true;
}

void batteryAbove()
{
  updateZEncoder();
  const XYMotion::Pose start = xyMeasuredPose();
  if (!xyCalibrated || !zHomed || !zEncoderOK
      || xySignX!=-1 || xySignY!=1 || fabs(start.a-5)>1 || fabs(start.b+90)>1
      || fabs(batteryZAngle())>0.5f) {
    Serial.println(F("ABOVE: no motion; requires photo start pose, xycal 5 -90 -1 1, homez"));
    return;
  }
  const XYMotion::Point target = XYMotion::forward(BATTERY_ABOVE);
  if (!batteryPathValid(start)) {
    Serial.println(F("ABOVE: no motion; path rejected")); return;
  }
  xyOperatorStopped = false;
  batteryHadWarning = false;
  if (xyStopRequested()) { Serial.println(F("ABOVE: cancelled before motion")); return; }
  gripper.write(OPEN_ANGLE);
  Serial.println(F("ABOVE: synchronized XYZ to target; XY 8 mm/s; ! stops; no descent or close"));
  if (!xyWaitMicros(500000UL) || !batterySynchronizedMove(start)) {
    digitalWrite(X_PUL,LOW); digitalWrite(Y_PUL,LOW); digitalWrite(Z_PUL,LOW);
    Serial.println(F("ABOVE PAUSED: sequence ended; inspect pos/e, no automatic retry")); return;
  }
  const float endpointError = xyEndpointError(xyMeasuredPose(), target);
  if (endpointError > XY_ENDPOINT_TOLERANCE_MM) batteryHadWarning = true;
  Serial.print(F("ABOVE XY endpoint error mm: ")); Serial.println(endpointError);
  Serial.println(batteryHadWarning
    ? F("ABOVE FINISHED WITH WARNING: manually align before descending")
    : F("ABOVE DONE: verify battery alignment before descending"));
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
    Serial.println(F("ABOVE: A 29.097 B -54.765; forward 174.087 up -39.659 mm"));
    Serial.println(F("PICK: A 14.619 B -56.309; forward 182.681 up -69.559 mm"));
    Serial.print(F("Saved Z DEG: ")); Serial.println(batteryAboveZ);
    Serial.println(F("abovez DEG: optional home-relative Z override; not a motor command")); return true;
  }
  if (input.startsWith("abovez ")) {
    float value;
    if (!xyParseNumbers(input,&value,1) || value<0 || value>45) {
      Serial.println(F("ERROR: abovez requires confirmed home-relative angle 0..45 DEG")); return true;
    }
    batteryAboveZ=value;
    Serial.println(F("ABOVE Z overridden; no motion; reset restores 28.056 DEG")); return true;
  }
  if (input.startsWith("above") || input.startsWith("startpose") || input.startsWith("pickinfo")) {
    Serial.println(F("ERROR: use startpose, abovez DEG, above, pickinfo")); return true;
  }
  return false;
}
#endif
