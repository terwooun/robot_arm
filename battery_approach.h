#ifndef ROBOT_ARM_BATTERY_APPROACH_H
#define ROBOT_ARM_BATTERY_APPROACH_H

// Recorded encoder-derived poses, not universal mechanical coordinates.
const XYMotion::Pose BATTERY_ABOVE = {29.097f, -54.765f};
// Reconstructed from last pos + three encoder-derived adjustments;
// power was switched off before a final pos/e snapshot. Recheck physically.
const XYMotion::Pose BATTERY_PICK = {14.390811f, -56.663719f};
const XYMotion::Pose BATTERY_PLACE = {21.276f, -57.929f};
const float BATTERY_PLACE_Z = 50.081f;
// Reconstructed from the final position minus later encoder-derived deltas.
const float BATTERY_CARRY_UP = -12.027f;
// Second pickup retains +10 mm; drop/regrasp retract 9 mm from prior targets.
// Heights and Z targets are unchanged.
const XYMotion::Point BATTERY_SECOND_PICK = {174.613f,-72.974f};
const XYMotion::Point BATTERY_SECOND_PLACE = {181.032f,-62.623f};
// No final pos/e at these two points: reconstructed from measured deltas.
const XYMotion::Point BATTERY_SECOND_REPICK = {182.412f,-67.436f};
const XYMotion::Point BATTERY_SECOND_RETURN = {174.766f,-60.441f};
// First-battery regrasp reconstructed after the completed RUN marker.
const XYMotion::Point BATTERY_FIRST_REPICK = {170.289f,-72.596f};
const XYMotion::Point BATTERY_FIRST_RETURN = {185.543f,-62.871f};
const XYMotion::Pose BATTERY_START = {5.0f,-90.0f};
float batteryAboveZ = 28.056f;
// Home-relative Z confirmed by the latest homez + manual alignment log.
// Default is restored after reset; abovez is an optional session override.
// Count elapsed planning/logging time toward SG90 travel, without duplicate waits.
const unsigned long BATTERY_GRIP_SETTLE_US = 500000UL;
unsigned long batteryGripChangedUs = 0;
int batteryGripAngle = -1;
void batteryCommandGrip(int angle)
{
  if(batteryGripAngle!=angle || gripper.read()!=angle) {
    batteryGripChangedUs=micros();
    batteryGripAngle=angle;
  }
  gripper.write(angle);
}
bool batteryAwaitGrip()
{
  if(batteryGripAngle!=gripper.read()) {
    batteryGripAngle=gripper.read();batteryGripChangedUs=micros();
  }
  const unsigned long elapsed=micros()-batteryGripChangedUs;
  if(xyStopRequested()) return false;
  return elapsed>=BATTERY_GRIP_SETTLE_US || xyWaitMicros(BATTERY_GRIP_SETTLE_US-elapsed);
}

// Closed or intermediate gripper commands retain the loaded profile.
// Relative to the preceding version: loaded XY 40->80, empty XY 80->240 mm/s.
float batterySpeedScale() { return gripper.read()==OPEN_ANGLE ? 6.0f : 2.0f; }
float batteryZSpeedScale() { return gripper.read()==OPEN_ANGLE ? 3.0f : 2.0f; }
XYMotion::Point batteryForwardOffset(const XYMotion::Point &p,float offset)
{
  return XYMotion::Point{p.forward+offset,p.up};
}

bool batteryHadWarning = false;
bool batteryStageCompleted = false;

float batteryZAngle()
{
  return zEncoderCount * (float)Z_ENC_SIGN * 360.0f / AS5600_CPR / GEAR_RATIO;
}

// Joint-space interpolation: all three axes share one DDA progress variable.
// The wrist follows a curved path, not the previous lift/rotate/advance route.
bool batteryPathValid(const XYMotion::Pose &start, const XYMotion::Pose &goal = BATTERY_ABOVE)
{
  for (int i=0; i<=200; ++i) {
    const float f = i/200.0f;
    const XYMotion::Pose p = {start.a+(goal.a-start.a)*f,
                              start.b+(goal.b-start.b)*f};
    if (!xyWithinWindow(p,start)) return false;
  }
  return true;
}

bool batterySynchronizedMove(const XYMotion::Pose &start,
                             const XYMotion::Pose &goal = BATTERY_ABOVE, bool rotateZ = true,
                             float zGoal = batteryAboveZ)
{
  const float speedScale=batterySpeedScale();
  long beforeX,beforeY;
  xyReadCounts(beforeX,beforeY);
  const float targetZ=rotateZ ? zGoal : batteryZAngle();
  const float dz=rotateZ ? targetZ-batteryZAngle() : 0.0f;
  if (!isfinite(dz) || fabs(dz)>45 || !zEncoderOK) return false;
  const long sx=lround((goal.b-start.b)*xySignX*PULSE_PER_DEGREE);
  const long sy=lround((goal.a-start.a)*xySignY*PULSE_PER_DEGREE);
  const long sz=lround(dz*PULSE_PER_DEGREE);
  const long nx=labs(sx), ny=labs(sy), nz=labs(sz);
  const long xyTicks=nx>ny ? nx:ny;
  const long ticks=xyTicks>nz ? xyTicks:nz;
  if (!ticks) return true;
  digitalWrite(X_PUL,LOW); digitalWrite(Y_PUL,LOW); digitalWrite(Z_PUL,LOW);
  digitalWrite(X_DIR,(sx>=0)!=X_DIR_INVERT ? HIGH:LOW);
  digitalWrite(Y_DIR,(sy>=0)!=Y_DIR_INVERT ? HIGH:LOW);
  digitalWrite(Z_DIR,(sz>=0)!=Z_DIR_INVERT ? HIGH:LOW);
  if (!xyWaitMicros(100UL)) return false;
  long ex=ticks/2,ey=ticks/2,ez=ticks/2;
  const float da=goal.a-start.a, db=goal.b-start.b;
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
    float speed=fmin(XY_FEED_MM_S*speedScale,sqrt(2.0f*XY_ACCEL_MM_S2*speedScale*speedScale*edge));
    speed=fmax(0.25f,speed);
    unsigned long halfPeriod=(unsigned long)(stepDistance/speed*500000.0f);
    // Z pulse limit follows load state independently of the XY multiplier.
    const unsigned long minimum=nz ? (unsigned long)(4000UL/batteryZSpeedScale()) : (unsigned long)(START_DELAY_US/speedScale);
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
  if(!xyWaitMicros(50000UL)) return false;
  updateZEncoder();
  if(!zEncoderOK) return false;
  const bool small=xyFeedbackOK(beforeX,beforeY,sx,sy,XY_FINAL_ERROR_DEG);
  if(small) {
    if(!xyCorrectEndpoint(goal,XYMotion::forward(goal),start))
      batteryHadWarning=true;
    if(xyOperatorStopped) return false;
  } else batteryHadWarning=true;
  const float error=batteryZAngle()-targetZ;
  if(!isfinite(error)) return false;
  if(fabs(error)>0.25f) {
    batteryHadWarning=true;
    Serial.print(F("WARNING: AUTO Z angle error DEG: "));Serial.println(error);
  }
  return true;
}

void batteryAbove()
{
  batteryStageCompleted=false;
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
  batteryCommandGrip(OPEN_ANGLE);
  Serial.println(F("ABOVE: synchronized XYZ to target; XY empty 240 / loaded 80 mm/s; Z empty 3x / loaded 2x; ! stops; no descent or close"));
  if (!batteryAwaitGrip() || !batterySynchronizedMove(start)) {
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
  batteryStageCompleted=true;
}

// One synchronized XY move to the corrected pick pose; Z stays stationary.
void batteryPick()
{
  batteryStageCompleted=false;
  updateZEncoder();
  const XYMotion::Pose start=xyMeasuredPose();
  if(!xyCalibrated || !zHomed || !zEncoderOK || xySignX!=-1 || xySignY!=1
      || xyEndpointError(start,XYMotion::forward(BATTERY_ABOVE))>3.0f
      || fabs(batteryZAngle()-27.957f)>0.5f) {
    Serial.println(F("PICK: no motion; requires aligned above position and Z near 27.957 DEG"));
    return;
  }
  if(!batteryPathValid(start,BATTERY_PICK)) {
    Serial.println(F("PICK: no motion; joint path rejected"));return;
  }
  xyOperatorStopped=false;
  batteryHadWarning=false;
  if(xyStopRequested()) {Serial.println(F("PICK PAUSED: cancelled before motion"));return;}
  batteryCommandGrip(OPEN_ANGLE);
  Serial.println(F("PICK: open, synchronized XY to target, close; Z held; no lift; ! stops"));
  const bool ok=batteryAwaitGrip() && batterySynchronizedMove(start,BATTERY_PICK,false);
  updateZEncoder();
  if(!ok || xyStopRequested() || !zEncoderOK) {
    digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
    Serial.println(F("PICK PAUSED: close skipped; inspect pos/e"));return;
  }
  const float error=xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_PICK));
  if(error>XY_ENDPOINT_TOLERANCE_MM) batteryHadWarning=true;
  Serial.print(F("PICK XY endpoint error mm: "));Serial.println(error);
  batteryCommandGrip(CLOSE_ANGLE);
  Serial.println(batteryHadWarning ? F("PICK FINISHED WITH WARNING: close commanded; verify grip; no lift")
                         : F("PICK DONE: close commanded; verify grip; no lift"));
  xyPrintPosition();
  batteryStageCompleted=true;
}

// Check a straight wrist segment before any transport motion.
bool batteryStraightValid(const XYMotion::Pose &start,const XYMotion::Point &target,
                          XYMotion::Pose &end)
{
  if(!XYMotion::straightPathValid(start,target,XY_SEGMENT_MM)) return false;
  const XYMotion::Point origin=XYMotion::forward(start);
  const int n=(int)ceil(hypot(target.forward-origin.forward,target.up-origin.up)/XY_SEGMENT_MM);
  XYMotion::Pose previous=start;
  for(int i=1;i<=n;++i) {
    const float f=(float)i/n;
    const XYMotion::Point p={origin.forward+(target.forward-origin.forward)*f,
                             origin.up+(target.up-origin.up)*f};
    XYMotion::Pose next;
    if(!XYMotion::inverse(p,previous,next) || !xyWithinWindow(next,start)) return false;
    previous=next;
  }
  end=previous;
  return true;
}

bool batteryStraightMove(const XYMotion::Point &target)
{
  const XYMotion::Point origin=XYMotion::forward(xyMeasuredPose());
  const float dx=target.forward-origin.forward,dy=target.up-origin.up;
  const float distance=hypot(dx,dy);
  if(!isfinite(distance)) return false;
  if(distance<=XY_ENDPOINT_TOLERANCE_MM) return true;
  if(distance>XY_AUTO_MAX_MOVE_MM) return false;
  if(xyStopRequested() || !zEncoderOK || !xyCalibrated) return false;
  // One continuous straight move: one acceleration ramp, one final settle/
  // correction phase. Internal 0.5mm IK samples do not stop the trajectory.
  xyMove(dx,dy,false,true,batterySpeedScale());
  if(!xyLastMoveCompleted || !zEncoderOK) return false;
  if(!xyLastMoveVerified) batteryHadWarning=true;
  return true;
}

void batteryPlace()
{
  batteryStageCompleted=false;
  updateZEncoder();
  const XYMotion::Pose start=xyMeasuredPose();
  if(!xyCalibrated || !zHomed || !zEncoderOK || xySignX!=-1 || xySignY!=1
      || xyEndpointError(start,XYMotion::forward(BATTERY_PICK))>5.0f
      || fabs(batteryZAngle()-27.957f)>0.5f || gripper.read()!=CLOSE_ANGLE) {
    Serial.println(F("PLACE: no motion; requires closed gripper at taught pick pose with Z homed"));return;
  }
  const XYMotion::Point drop=XYMotion::forward(BATTERY_PLACE);
  const XYMotion::Point lift={XYMotion::forward(start).forward-5.0f,BATTERY_CARRY_UP};
  const XYMotion::Point carry={drop.forward,BATTERY_CARRY_UP};
  XYMotion::Pose lifted,carried,lowered;
  if(!batteryStraightValid(start,lift,lifted)
      || !batteryStraightValid(lifted,carry,carried)
      || !batteryStraightValid(carried,drop,lowered)
      || fabs(BATTERY_PLACE_Z-batteryZAngle())>45) {
    Serial.println(F("PLACE: no motion; transport path rejected"));return;
  }
  xyOperatorStopped=false;batteryHadWarning=false;
  Serial.println(F("PLACE: lift -> rotate -> transfer -> lower -> open; ! stops; no retreat"));
  bool ok=batteryAwaitGrip(); // Allow the preceding close command to settle.
  if(ok) {
    Serial.println(F("PLACE: LIFT continuous XY"));
    ok=batteryStraightMove(lift);
  }
  if(ok) {
    Serial.println(F("PLACE: ROTATE"));
    const XYMotion::Pose p=xyMeasuredPose();
    ok=batterySynchronizedMove(p,p,true,BATTERY_PLACE_Z);
  }
  if(ok) {
    Serial.println(F("PLACE: TRANSFER"));
    const XYMotion::Point p=XYMotion::forward(xyMeasuredPose());
    ok=batteryStraightMove(XYMotion::Point{drop.forward,p.up});
  }
  if(ok) {
    Serial.println(F("PLACE: LOWER continuous XY"));
    ok=batteryStraightMove(drop);
  }
  updateZEncoder();
  if(!ok || xyStopRequested() || !zEncoderOK) {
    digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
    Serial.println(F("PLACE PAUSED: release skipped; gripper remains closed"));return;
  }
  const float error=xyEndpointError(xyMeasuredPose(),drop);
  if(error>XY_ENDPOINT_TOLERANCE_MM) batteryHadWarning=true;
  Serial.print(F("PLACE XY endpoint error mm: "));Serial.println(error);
  batteryCommandGrip(OPEN_ANGLE);
  Serial.println(batteryHadWarning ? F("PLACE FINISHED WITH WARNING: open commanded; no retreat")
                                  : F("PLACE DONE: open commanded; no retreat"));
  xyPrintPosition();batteryStageCompleted=true;
}

// One taught station: lift first, combine high-level XY/Z relocation, descend.
// Repeated tuning commands are collapsed into a single target for each phase.
bool batteryVisit(const XYMotion::Point &target,float targetZ,float carryUp,bool closeAfter,bool rotateZ=true)
{
  updateZEncoder();
  if(!xyCalibrated || !zHomed || !zEncoderOK || xySignX!=-1 || xySignY!=1
      || gripper.read()!=(closeAfter ? OPEN_ANGLE:CLOSE_ANGLE)) return false;
  const XYMotion::Pose start=xyMeasuredPose();
  const XYMotion::Point origin=XYMotion::forward(start);
  const XYMotion::Point lift={origin.forward,carryUp};
  const XYMotion::Point carry={target.forward,carryUp};
  XYMotion::Pose lifted,carried,lowered;
  if(carryUp<origin.up || carryUp<target.up || fabs(targetZ-batteryZAngle())>45
      || !batteryStraightValid(start,lift,lifted)
      || !XYMotion::inverse(carry,lifted,carried) || !batteryPathValid(lifted,carried)
      || !batteryStraightValid(carried,target,lowered)) {
    Serial.println(F("STATION: no motion; path rejected"));return false;
  }
  if(xyStopRequested() || !batteryAwaitGrip()) return false;
  Serial.println(F("STATION: LIFT"));
  if(!batteryStraightMove(lift)) return false;
  // Recompute from measured arrival, retain the same target instead of replaying deltas.
  const XYMotion::Pose actualLift=xyMeasuredPose();
  if(!XYMotion::inverse(carry,actualLift,carried) || !batteryPathValid(actualLift,carried)) return false;
  Serial.println(F("STATION: TRANSFER XYZ"));
  if(!batterySynchronizedMove(actualLift,carried,rotateZ,targetZ)) return false;
  Serial.println(F("STATION: LOWER XY"));
  if(!batteryStraightMove(target)) return false;
  updateZEncoder();
  if(xyStopRequested() || !zEncoderOK) return false;
  if(xyEndpointError(xyMeasuredPose(),target)>XY_ENDPOINT_TOLERANCE_MM) batteryHadWarning=true;
  batteryCommandGrip(closeAfter ? CLOSE_ANGLE:OPEN_ANGLE);
  Serial.println(closeAfter ? F("STATION: CLOSE commanded") : F("STATION: OPEN commanded"));
  xyPrintPosition();
  return true;
}

template <typename T>
void batteryRunPause(T message)
{
  digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
  batteryStageCompleted=false;
  Serial.println(message);
}

void batteryRun()
{
  batteryStageCompleted=false;
  const XYMotion::Pose start=xyMeasuredPose();
  if(!xyCalibrated || xySignX!=-1 || xySignY!=1
      || fabs(start.a-5)>1 || fabs(start.b+90)>1) {
    Serial.println(F("RUN PAUSED at above: requires photo start pose and s; no motion"));return;
  }
  xyOperatorStopped=false;
  if(xyStopRequested()) return;
  Serial.println(F("RUN: homez -> battery1 -> battery2 -> battery2 return -> battery1 return -> repeat; ! stops"));
  homeZ();
  if(!zHomed || !zEncoderOK || zHomeCancelled) {
    batteryRunPause(F("RUN PAUSED at homez; no approach"));return;
  }
  bool warning=false;
  batteryAbove();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at above"));return;}
  warning=warning || batteryHadWarning;
  if(xyStopRequested()) {batteryRunPause(F("RUN PAUSED before pick"));return;}
  batteryPick();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at pick"));return;}
  warning=warning || batteryHadWarning;
  if(xyStopRequested()) {batteryRunPause(F("RUN PAUSED before place"));return;}
  batteryPlace();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at place"));return;}
  warning=warning || batteryHadWarning;
  bool repeat=false;
  float repeatForwardOffset=0.0f; // Fixed +1.2 mm from cycle two.
  float repeatZOffset=0.0f; // Fixed +0.4 from cycle two, never accumulated.
  while(true) {
    batteryHadWarning=false;
    Serial.println(F("RUN: SECOND PICK"));
    // Empty-gripper retreat uses the established high clearance, not low tuning positions.
    if(!batteryVisit(batteryForwardOffset(repeat ? BATTERY_SECOND_RETURN : BATTERY_SECOND_PICK,repeatForwardOffset),
                     (repeat ? 38.216f : 38.859f)+repeatZOffset,BATTERY_CARRY_UP,true)) {
      batteryRunPause(F("RUN PAUSED at second pick"));return;
    }
    Serial.println(F("RUN: SECOND PLACE"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_SECOND_PLACE,repeatForwardOffset),61.526f+repeatZOffset,-12.704f,false)) {
      batteryRunPause(F("RUN PAUSED at second place"));return;
    }
    Serial.println(F("RUN: SECOND REPICK (reconstructed target)"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_SECOND_REPICK,repeatForwardOffset),61.526f+repeatZOffset,-12.718f,true,false)) {
      batteryRunPause(F("RUN PAUSED at second repick"));return;
    }
    Serial.println(F("RUN: SECOND RETURN (reconstructed target)"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_SECOND_RETURN,repeatForwardOffset),38.216f+repeatZOffset,-7.590f,false)) {
      batteryRunPause(F("RUN PAUSED at second return"));return;
    }
    Serial.println(F("RUN: FIRST REPICK (reconstructed target)"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_FIRST_REPICK,repeatForwardOffset),51.243f+repeatZOffset,-15.570f,true)) {
      batteryRunPause(F("RUN PAUSED at first repick"));return;
    }
    Serial.println(F("RUN: FIRST RETURN"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_FIRST_RETURN,repeatForwardOffset),26.442f+repeatZOffset,-17.718f,false)) {
      batteryRunPause(F("RUN PAUSED at first return"));return;
    }
    Serial.println(warning || batteryHadWarning ? F("RUN CYCLE FINISHED WITH WARNING; repeating; ! stops")
                                               : F("RUN CYCLE DONE; repeating; ! stops"));
    // Regrasp at the actual return target, not the old photo-pose pickup target.
    // batteryVisit lifts the empty gripper before descending to grasp again.
    if(xyStopRequested()) {batteryRunPause(F("RUN STOPPED by !"));return;}
    repeatZOffset=0.4f;
    repeatForwardOffset=1.2f;
    Serial.println(F("RUN: FORWARD OFFSET +1.200 mm (fixed, not cumulative)"));
    Serial.println(F("RUN: Z OFFSET +0.400 DEG (fixed for all repeat targets)"));
    Serial.println(F("RUN: REPEAT FIRST PICK"));
    if(!batteryVisit(batteryForwardOffset(BATTERY_FIRST_RETURN,repeatForwardOffset),26.442f+repeatZOffset,-17.718f,true)) {
      batteryRunPause(F("RUN PAUSED at repeat first pick"));return;
    }
    Serial.println(F("RUN: REPEAT FIRST PLACE"));
    if(!batteryVisit(batteryForwardOffset(XYMotion::forward(BATTERY_PLACE),repeatForwardOffset),BATTERY_PLACE_Z+repeatZOffset,BATTERY_CARRY_UP,false)) {
      batteryRunPause(F("RUN PAUSED at repeat first place"));return;
    }
    repeat=true;
    warning=batteryHadWarning;
  }

}

bool batteryHandleCommand(const String &input)
{
  if (input == "s" || input == "startpose") {
    xyHandleCommand(String("xycal 5 -90 -1 1"));
    Serial.println(F("STARTPOSE: manual photo pose registered; no motion")); return true;
  }
  if (input == "run") { batteryRun(); return true; }
  if (input == "place") { batteryPlace(); return true; }
  if (input == "pick") { batteryPick(); return true; }
  if (input == "above") { batteryAbove(); return true; }
  if (input == "pickinfo") {
    Serial.println(F("ABOVE: A 29.097 B -54.765; forward 174.087 up -39.659 mm"));
    const XYMotion::Point pick=XYMotion::forward(BATTERY_PICK);
    Serial.print(F("PICK: A "));Serial.print(BATTERY_PICK.a,3);
    Serial.print(F(" B "));Serial.print(BATTERY_PICK.b,3);
    Serial.print(F("; forward "));Serial.print(pick.forward,3);
    Serial.print(F(" up "));Serial.print(pick.up,3);
    Serial.println(F(" mm; close 20 DEG; Z unchanged"));
    Serial.println(F("PLACE: A 21.276 B -57.929; Z 50.081 DEG; open 100 DEG"));
    Serial.println(F("CARRY: UP -12.027 mm; first lift also retracts 5 mm"));
    Serial.println(F("SECOND PICK: F 174.613 U -72.974 Z 38.859 (+10 mm forward)"));
    Serial.println(F("SECOND PLACE: F 181.032 U -62.623 Z 61.526 (+1.0 mm from previous target)"));
    Serial.println(F("SECOND REPICK: F 182.412 U -67.436; Z held from second place (no rotation)"));
    Serial.println(F("SECOND RETURN: F 174.766 U -60.441 Z 38.216 (reconstructed)"));
    Serial.println(F("FIRST REPICK: F 170.289 U -72.596 Z 51.243 (reconstructed)"));
    Serial.println(F("FIRST RETURN: F 185.543 U -62.871 Z 26.442; lift and repeat until !"));
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
  if (input.startsWith("above") || input.startsWith("startpose") || input.startsWith("pick") || input.startsWith("place") || input.startsWith("run")) {
    Serial.println(F("ERROR: use s, above, pick, place, run, pickinfo, abovez DEG")); return true;
  }
  return false;
}
#endif
