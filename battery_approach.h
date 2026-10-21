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
const XYMotion::Point BATTERY_SECOND_PLACE = {179.432f,-62.623f};
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
// Avoid waiting the full SG90 time twice at each stage boundary.  The servo
// still needs a small real travel time: zero wait can drop or knock batteries.
const unsigned long BATTERY_GRIP_OPEN_SETTLE_US = 120000UL;
const unsigned long BATTERY_GRIP_CLOSE_SETTLE_US = 160000UL;
unsigned long batteryServoCommandUs = 0;
int batteryServoLastAngle = -1;
float batteryDiagonalBoost = 1.0f; // 5x selectable after no-load checks
// Slow down loaded arm movements only. Large Y-axis lag was measured on lift.
float batteryLoadedSpeedMax = 1.5f;
// Final returned battery has shown Y-axis lag during its loaded vertical lift.
// Limit ONLY this lift to a torque-friendly speed. Do not relax the 1 DEG
// in-motion encoder stop or the 1 mm station endpoint guard.
float batteryFinalLiftSpeedCap = 0.75f;
// 0 = continuous until !; 1 or more = test with a finite number of cycles.
unsigned int batteryCycleLimit = 0;
const float BATTERY_REPEAT_ABOVE_UP = -39.660f; // same as BATTERY_ABOVE height
const float BATTERY_RELEASE_ENDPOINT_MAX_ERROR_MM = 1.5f;

void batteryTrackGrip()
{
  const int current=gripper.read();
  if(batteryServoLastAngle < 0) {
    batteryServoLastAngle=current;
    batteryServoCommandUs=micros()-BATTERY_GRIP_CLOSE_SETTLE_US;
  } else if (current!=batteryServoLastAngle) {
    // Also detects manual 'open'/'close' commands outside battery automation.
    batteryServoLastAngle=current;
    batteryServoCommandUs=micros();
  }
}

void batteryCommandGrip(int angle)
{
  batteryTrackGrip();
  if(gripper.read()==angle) return; // Do not restart the servo settle clock.
  gripper.write(angle);
  batteryServoLastAngle=angle;
  batteryServoCommandUs=micros();
}

bool batteryAwaitGrip()
{
  batteryTrackGrip();
  const unsigned long settle=(batteryServoLastAngle==CLOSE_ANGLE)
      ? BATTERY_GRIP_CLOSE_SETTLE_US : BATTERY_GRIP_OPEN_SETTLE_US;
  const unsigned long elapsed=(unsigned long)(micros()-batteryServoCommandUs);
  // Path planning, logging, and any other time already elapsed are credited.
  return elapsed>=settle || xyWaitMicros(settle-elapsed);
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
  long beforeX,beforeY;
  xyReadCounts(beforeX,beforeY);
  // Fast empty approach is unchanged; loaded XYZ moves use a torque-friendly cap.
  const float commandedSpeed = robotSpeedScale;
  const float motionSpeed = (gripper.read() == CLOSE_ANGLE)
    ? fmin(commandedSpeed, batteryLoadedSpeedMax) : commandedSpeed;
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
  // Hardware DIR setup only; remove old extra 20 ms stop at every XYZ move.
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
  // Preserve the original joint-space trajectory and DDA pulse counts.
  // Estimate distance per DDA tick from the validated 200-sample wrist arc.
  // This avoids repeated sin/cos/hypot calculations in every motor pulse.
  const float stepDistance=length/(float)ticks;
  for(long i=0;i<ticks;++i) {
    if(xyStopRequested() || (sz<0 && digitalRead(Z_HOME)==HIGH)) return false;
    const float along=(i+0.5f)*stepDistance;
    const float edge=fmax(0.0f,fmin(along,length-along));
    float speed=fmin(XY_FEED_MM_S*motionSpeed,sqrt(2.0f*XY_ACCEL_MM_S2*motionSpeed*motionSpeed*edge));
    speed=fmax(0.25f,speed);
    unsigned long halfPeriod=(unsigned long)(stepDistance/speed*500000.0f);
    // Scale the XY/XYZ pulse floor; Z homing speed remains unchanged.
    const unsigned long minimum=nz
      ? (unsigned long)(4000.0f / motionSpeed)
      : (unsigned long)(START_DELAY_US / motionSpeed);
    if(halfPeriod<minimum) halfPeriod=minimum;
    ex+=nx; ey+=ny; ez+=nz;
    const bool px=ex>=ticks, py=ey>=ticks, pz=ez>=ticks;
    if(px) {ex-=ticks;digitalWrite(X_PUL,HIGH);}
    if(py) {ey-=ticks;digitalWrite(Y_PUL,HIGH);}
    if(pz) {ez-=ticks;digitalWrite(Z_PUL,HIGH);}
    const bool keepMoving=xyWaitMicros(halfPeriod);
    digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
    if(!keepMoving || !xyWaitMicros(halfPeriod)) return false;
    if(i%16==0) {updateZEncoder();if(!zEncoderOK) return false;}
  }
  updateZEncoder();
  if(!zEncoderOK) return false;
  // Encoder readings immediately after the last pulse can still be settling.
  // Only 6 ms, with '!'/stop processing; no long post-move dwell.
  if(!xyWaitMicros(6000UL)) return false;
  const bool small=xyFeedbackOK(beforeX,beforeY,sx,sy,XY_FINAL_ERROR_DEG);
  if(small) {
    // Correct small motor/encoder following errors at LOW pulse rate.
    // Limit remains 6 pulses/attempt, 15 pulses/axis in total.
    if(!xyCorrectEndpoint(goal,XYMotion::forward(goal),start))
      batteryHadWarning=true;
    if(xyOperatorStopped) return false;
  } else {
    batteryHadWarning=true;
    if (motionSpeed > 2.0f) {
      Serial.println(F("HIGH SPEED STOP: XY following error above correction window"));
      return false;
    }
  }
  // Even if an endpoint correction fails, do not enter high-speed pick/place
  // when the actual gripper XY target is materially displaced.
  const float finalXYError=xyEndpointError(xyMeasuredPose(),XYMotion::forward(goal));
  if(!isfinite(finalXYError)) return false;
  if(finalXYError > XY_ENDPOINT_TOLERANCE_MM) {
    batteryHadWarning=true;
    Serial.print(F("WARNING: XY endpoint residual mm = "));
    Serial.print(finalXYError,3); Serial.println();
  }
  if(motionSpeed>2.0f && finalXYError>1.0f) {
    Serial.println(F("HIGH SPEED STOP: XY endpoint residual exceeds 1 mm"));
    return false;
  }
  const float error=batteryZAngle()-targetZ;
  if(!isfinite(error)) return false;
  if(fabs(error)>0.25f) {
    batteryHadWarning=true;
    Serial.print(F("WARNING: AUTO Z angle error DEG: "));Serial.println(error);
    if (robotSpeedScale > 2.0f && fabs(error)>1.0f) {
      Serial.println(F("HIGH SPEED STOP: Z following error"));
      return false;
    }
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
  Serial.println(F("ABOVE: synchronized XYZ to target; ! stops; no descent or close"));
  if(!batteryAwaitGrip()) { Serial.println(F("ABOVE: gripper not ready")); return; }
  // diagboost 5 at speed 3 gives 15x command scale (OPTIONAL ONLY).
  // Explicit opt-in: high-speed diagonal can stall the 12V Z stepper.
  const float originalSpeed=robotSpeedScale;
  const float boostedSpeed=fmin(15.0f,originalSpeed*batteryDiagonalBoost);
  robotSpeedScale=boostedSpeed;
  const bool moved=batterySynchronizedMove(start);
  robotSpeedScale=originalSpeed;
  if (!moved) {
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
  const float requestedSpeed = robotSpeedScale;
  const bool loaded = gripper.read() == CLOSE_ANGLE;
  if (loaded && requestedSpeed > batteryLoadedSpeedMax) {
    robotSpeedScale = batteryLoadedSpeedMax;
    Serial.print(F("LOADED XY SPEED CAP: "));
    Serial.print(robotSpeedScale, 1);
    Serial.println(F("x"));
  }
  xyMove(dx,dy,false,true);
  robotSpeedScale = requestedSpeed;
  if(!xyLastMoveCompleted || !zEncoderOK) return false;
  // A completed pulse train is not proof the loaded wrist reached its target.
  // Stop the station before Z rotation/release if the measured target is missed.
  const float residual = xyEndpointError(xyMeasuredPose(),target);
  if (!isfinite(residual) || residual > 1.0f) {
    Serial.print(F("STATION STOP: XY endpoint miss "));
    Serial.print(residual, 3);
    Serial.println(F(" mm; stop before next operation"));
    xyInvalidateReference();
    return false;
  }
  if(!xyLastMoveVerified) batteryHadWarning=true;
  return true;
}

// Temporarily cap speed for the last loaded lift; always restore the
// operator's selected speed even if a safety stop aborts the move.
bool batteryFinalLiftMove(const XYMotion::Point &target)
{
  if (!isfinite(batteryFinalLiftSpeedCap) || batteryFinalLiftSpeedCap <= 0.0f)
    return false;
  const float selectedSpeed = robotSpeedScale;
  robotSpeedScale = fmin(selectedSpeed, batteryFinalLiftSpeedCap);
  Serial.print(F("FINAL BATTERY LIFT SPEED CAP: "));
  Serial.print(robotSpeedScale, 2);
  Serial.println(F("x"));
  const bool completed = batteryStraightMove(target);
  robotSpeedScale = selectedSpeed;
  return completed;
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
  bool ok=batteryAwaitGrip(); // Wait only for any REMAINING servo travel time.
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
bool batteryVisit(const XYMotion::Point &target,float targetZ,float carryUp,bool closeAfter,
                  bool finalReturnLift = false)
{
  updateZEncoder();
  if(!xyCalibrated || !zHomed || !zEncoderOK || xySignX!=-1 || xySignY!=1
      || gripper.read()!=(closeAfter ? OPEN_ANGLE:CLOSE_ANGLE)) {
    Serial.println(F("STATION STOP: reference, Z encoder/home, or gripper state invalid"));
    return false;
  }
  const XYMotion::Pose start=xyMeasuredPose();
  const XYMotion::Point origin=XYMotion::forward(start);
  const XYMotion::Point lift={origin.forward,carryUp};
  const XYMotion::Point carry={target.forward,carryUp};
  XYMotion::Pose lifted,carried,lowered;
  if(carryUp<origin.up || carryUp<target.up || fabs(targetZ-batteryZAngle())>45
      || !batteryStraightValid(start,lift,lifted)
      || !XYMotion::inverse(carry,lifted,carried) || !batteryPathValid(lifted,carried)
      || !batteryStraightValid(carried,target,lowered)) {
    Serial.println(F("STATION STOP: lift/transfer/lower path or Z angle rejected"));
    Serial.print(F("FROM F/U: ")); Serial.print(origin.forward,2);Serial.print('/');Serial.print(origin.up,2);Serial.println();
    Serial.print(F("TO   F/U: ")); Serial.print(target.forward,2);Serial.print('/');Serial.print(target.up,2);Serial.println();
    return false;
  }
  if(xyStopRequested() || !batteryAwaitGrip()) return false;
  // The only remaining servo wait is physical release/grip travel.
  // After it completes, raise immediately with no additional dwell.
  Serial.println(F("STATION: LIFT"));
  const bool liftComplete = finalReturnLift
    ? batteryFinalLiftMove(lift) : batteryStraightMove(lift);
  if(!liftComplete) {
    if(finalReturnLift)
      Serial.println(F("FINAL LIFT STOP: inspect Y motor torque, driver current, battery clearance and encoder wiring"));
    Serial.println(F("STATION STOP: lift failed"));
    return false;
  }
  // Recompute from measured arrival, retain the same target instead of replaying deltas.
  const XYMotion::Pose actualLift=xyMeasuredPose();
  if(!XYMotion::inverse(carry,actualLift,carried) || !batteryPathValid(actualLift,carried)) {
    Serial.println(F("STATION STOP: transfer replan invalid"));return false;
  }
  Serial.println(F("STATION: TRANSFER XYZ"));
  if(!batterySynchronizedMove(actualLift,carried,true,targetZ)) {
    Serial.println(F("STATION STOP: XYZ transfer failed; check encoder warnings"));return false;
  }
  Serial.println(F("STATION: LOWER XY"));
  if(!batteryStraightMove(target)) {
    Serial.println(F("STATION STOP: lower failed; grip not changed"));return false;
  }
  updateZEncoder();
  if(xyStopRequested() || !zEncoderOK) {
    Serial.println(F("STATION STOP: operator stop or Z sensor lost"));return false;
  }
  const float positionError=xyEndpointError(xyMeasuredPose(),target);
  if(!isfinite(positionError) || positionError>BATTERY_RELEASE_ENDPOINT_MAX_ERROR_MM) {
    Serial.print(F("STATION STOP: grip/place alignment error mm = "));
    Serial.print(positionError,3);Serial.println();
    // Never force open if the robot has not actually reached the drop target.
    return false;
  }
  if(positionError>XY_ENDPOINT_TOLERANCE_MM) batteryHadWarning=true;
  batteryCommandGrip(closeAfter ? CLOSE_ANGLE:OPEN_ANGLE);
  Serial.println(closeAfter ? F("STATION: CLOSE commanded") : F("STATION: OPEN commanded"));
  // No multi-line position dump between release and the next vertical lift.
  // Encoder position checks remain active before gripping or releasing.
  return true;
}

// At the final returned battery, keep the wrist horizontal coordinate fixed.
// Lift to the ORIGINAL pre-pick height, come straight back down and regrasp.
// Z remains at the last-release angle. No relocate/rotate phase is executed.
bool batteryRegraspAtLastReturn()
{
  updateZEncoder();
  if (!xyCalibrated || !zHomed || !zEncoderOK || gripper.read()!=OPEN_ANGLE
      || xySignX!=-1 || xySignY!=1) {
    Serial.println(F("REPEAT STOP: home, encoders, or open grip invalid"));
    return false;
  }
  const XYMotion::Pose start=xyMeasuredPose();
  const XYMotion::Point current=XYMotion::forward(start);
  const float releaseError=xyEndpointError(start,BATTERY_FIRST_RETURN);
  if (!isfinite(releaseError) || releaseError>BATTERY_RELEASE_ENDPOINT_MAX_ERROR_MM
      || fabs(batteryZAngle()-26.442f)>0.5f) {
    Serial.println(F("REPEAT STOP: not aligned over last returned battery"));
    return false;
  }
  const XYMotion::Point raised={current.forward,BATTERY_REPEAT_ABOVE_UP};
  const XYMotion::Point lowered={current.forward,BATTERY_FIRST_RETURN.up};
  XYMotion::Pose highPose,lowPose;
  // Validate both directions before motion, with the same wrist X coordinate.
  if (!batteryStraightValid(start,raised,highPose)
      || !batteryStraightValid(highPose,lowered,lowPose)) {
    Serial.println(F("REPEAT STOP: vertical lift/descent path invalid"));
    return false;
  }
  // The SG90 must physically let go; elapsed planning time counts as wait.
  if (!batteryAwaitGrip() || xyStopRequested()) return false;
  Serial.println(F("REPEAT: VERTICAL LIFT AT LAST DROP"));
  if (!batteryStraightMove(raised)) {
    Serial.println(F("REPEAT STOP: vertical lift failed"));
    return false;
  }
  Serial.println(F("REPEAT: VERTICAL LOWER AT SAME X"));
  if (!batteryStraightMove(lowered)) {
    Serial.println(F("REPEAT STOP: vertical lower failed"));
    return false;
  }
  updateZEncoder();
  if (xyStopRequested() || !zEncoderOK
      || fabs(batteryZAngle()-26.442f)>0.5f
      || xyEndpointError(xyMeasuredPose(),BATTERY_FIRST_RETURN)>BATTERY_RELEASE_ENDPOINT_MAX_ERROR_MM) {
    Serial.println(F("REPEAT STOP: final XY/Z alignment not verified"));
    return false;
  }
  batteryCommandGrip(CLOSE_ANGLE);
  Serial.println(F("REPEAT: CLOSE commanded at last drop"));
  return true;
}

template <typename T>
void batteryRunPause(T message)
{
  digitalWrite(X_PUL,LOW);digitalWrite(Y_PUL,LOW);digitalWrite(Z_PUL,LOW);
  batteryStageCompleted=false;
  Serial.println(message);
}

// Empty-gripper return: lift clear, relocate above start, then lower.
// Keep the registered reference; never relabel a measured endpoint as zero.
bool batteryReturnStart()
{
  updateZEncoder();
  if(!xyCalibrated || !zHomed || !zEncoderOK || gripper.read()!=OPEN_ANGLE) return false;
  const XYMotion::Pose start=xyMeasuredPose();
  const XYMotion::Point origin=XYMotion::forward(start);
  const XYMotion::Point lift={origin.forward,-17.718f};
  const XYMotion::Point aboveStart={XYMotion::forward(BATTERY_START).forward,-35.0f};
  XYMotion::Pose lifted,above;
  if(!batteryStraightValid(start,lift,lifted)
     || !XYMotion::inverse(aboveStart,lifted,above)
     || !batteryPathValid(lifted,above) || !batteryPathValid(above,BATTERY_START)) return false;
  Serial.println(F("RUN: EMPTY LIFT"));
  if(!batteryAwaitGrip() || !batteryStraightMove(lift)) return false;
  Serial.println(F("RUN: RETURN ABOVE START XYZ"));
  if(!batteryPathValid(xyMeasuredPose(),above)
     || !batterySynchronizedMove(xyMeasuredPose(),above,true,0.0f)) return false;
  Serial.println(F("RUN: LOWER TO START XY"));
  if(!batteryPathValid(xyMeasuredPose(),BATTERY_START)
     || !batterySynchronizedMove(xyMeasuredPose(),BATTERY_START,false)) return false;
  return !xyStopRequested() && zEncoderOK;
}

void batteryRun()
{
  batteryStageCompleted=false;
  const XYMotion::Pose start=xyMeasuredPose();
  if(!xyCalibrated || xySignX!=-1 || xySignY!=1
      || fabs(start.a-5)>1 || fabs(start.b+90)>1) {
    Serial.println(F("RUN PAUSED: requires photo start pose and s; no motion"));return;
  }
  xyOperatorStopped=false;
  if(xyStopRequested()) return;
  Serial.println(F("RUN: homez, above, first pick/place, second transfer/return, first return; repeat until !"));
  homeZ();
  if(!zHomed || !zEncoderOK || zHomeCancelled) {
    batteryRunPause(F("RUN PAUSED at homez; no approach"));return;
  }
  batteryAbove();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at above"));return;}
  if(xyStopRequested()) {batteryRunPause(F("RUN STOPPED before first pick"));return;}
  batteryPick();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at first pick"));return;}
  if(xyStopRequested()) {batteryRunPause(F("RUN STOPPED before first place"));return;}
  batteryPlace();
  if(!batteryStageCompleted) {batteryRunPause(F("RUN PAUSED at first place"));return;}

  unsigned long completedCycles=0;
  bool repeat=false;
  while (!xyStopRequested()) {
    // On subsequent loops pick from the actual returned location, NOT the
    // initial station. The initial and returned battery coordinates differ.
    Serial.println(repeat ? F("RUN: SECOND PICK AT RETURNED POSITION")
                          : F("RUN: SECOND PICK"));
    if(!batteryVisit(repeat ? BATTERY_SECOND_RETURN : BATTERY_SECOND_PICK,
                     repeat ? 38.216f : 38.859f,BATTERY_CARRY_UP,true)) {
      batteryRunPause(F("RUN PAUSED at second pick"));return;
    }
    Serial.println(F("RUN: SECOND PLACE"));
    if(!batteryVisit(BATTERY_SECOND_PLACE,61.526f,-12.704f,false)) {
      batteryRunPause(F("RUN PAUSED at second place"));return;
    }
    Serial.println(F("RUN: SECOND REPICK"));
    if(!batteryVisit(BATTERY_SECOND_REPICK,62.473f,-12.718f,true)) {
      batteryRunPause(F("RUN PAUSED at second repick"));return;
    }
    Serial.println(F("RUN: SECOND RETURN"));
    if(!batteryVisit(BATTERY_SECOND_RETURN,38.216f,-7.590f,false)) {
      batteryRunPause(F("RUN PAUSED at second return"));return;
    }
    Serial.println(F("RUN: FIRST REPICK"));
    if(!batteryVisit(BATTERY_FIRST_REPICK,51.243f,-15.570f,true)) {
      batteryRunPause(F("RUN PAUSED at first repick"));return;
    }
    Serial.println(F("RUN: LAST BATTERY PLACE / FIRST RETURN"));
    if(!batteryVisit(BATTERY_FIRST_RETURN,26.442f,-17.718f,false,true)) {
      batteryRunPause(F("RUN PAUSED at last battery place. Read preceding STATION STOP / WARNING lines."));return;
    }
    completedCycles++;
    Serial.print(F("RUN: completed full return cycle #"));Serial.println(completedCycles);

    if(batteryCycleLimit && completedCycles>=batteryCycleLimit) {
      Serial.println(F("RUN: finite cycle limit reached; returning empty arm to start"));
      if(!batteryReturnStart()) {
        batteryRunPause(F("RUN PAUSED returning to start"));return;
      }
      batteryStageCompleted=true;
      Serial.println(F("RUN: cycles completed; arm at registered start pose"));
      return;
    }
    if(xyStopRequested()) {batteryRunPause(F("RUN STOPPED by !"));return;}
    // From the final returned drop: vertical rise, same-X descent, regrasp.
    // Do not call the general lift/XYZ-transfer/lower station planner here.
    Serial.println(F("RUN: REPEAT FIRST PICK; vertical up/down at last drop"));
    if(!batteryRegraspAtLastReturn()) {
      batteryRunPause(F("RUN PAUSED at repeat first pick"));return;
    }
    Serial.println(F("RUN: REPEAT FIRST PLACE"));
    if(!batteryVisit(XYMotion::forward(BATTERY_PLACE),BATTERY_PLACE_Z,BATTERY_CARRY_UP,false)) {
      batteryRunPause(F("RUN PAUSED at repeat first place"));return;
    }
    // At this point both batteries occupy the same stations as after the
    // initial first place. Repeat the second battery shuttle indefinitely.
    repeat=true;
  }
  batteryRunPause(F("RUN STOPPED by !"));
}

bool batteryHandleCommand(const String &input)
{
  if (input == "lastlift") {
    Serial.print(F("FINAL BATTERY LIFT SPEED CAP: "));
    Serial.print(batteryFinalLiftSpeedCap, 2);
    Serial.println(F("x"));
    return true;
  }
  if (input.startsWith("lastlift ")) {
    float value = 0.0f;
    if (xyParseNumbers(input, &value, 1)
        && (value == 0.5f || value == 0.75f || value == 1.0f)) {
      batteryFinalLiftSpeedCap = value;
      Serial.print(F("FINAL BATTERY LIFT SPEED CAP SET: "));
      Serial.print(batteryFinalLiftSpeedCap, 2);
      Serial.println(F("x"));
    } else {
      Serial.println(F("ERROR: lastlift 0.5 / 0.75 / 1"));
    }
    return true;
  }
  if (input == "loadspeed") {
    Serial.print(F("LOADED MOVE SPEED LIMIT: "));
    Serial.print(batteryLoadedSpeedMax, 1);
    Serial.println(F("x; empty moves retain the speed setting"));
    return true;
  }
  if (input.startsWith("loadspeed ")) {
    float value = 0.0f;
    if (xyParseNumbers(input, &value, 1)
        && (value == 1.0f || value == 1.5f || value == 2.0f)) {
      batteryLoadedSpeedMax = value;
      Serial.print(F("LOADED MOVE SPEED LIMIT SET: "));
      Serial.print(batteryLoadedSpeedMax, 1);
      Serial.println(F("x"));
    } else {
      Serial.println(F("ERROR: loadspeed 1 / 1.5 / 2"));
    }
    return true;
  }
  if (input == "s" || input == "startpose") {
    xyHandleCommand(String("xycal 5 -90 -1 1"));
    Serial.println(F("STARTPOSE: manual photo pose registered; no motion")); return true;
  }
  if (input == "cycles") {
    Serial.print(F("AUTOMATIC CYCLES: "));
    if(batteryCycleLimit==0) Serial.println(F("infinite until !"));
    else Serial.println(batteryCycleLimit);
    return true;
  }
  if(input.startsWith("cycles ")) {
    float count=0.0f;
    const bool valid=xyParseNumbers(input,&count,1) && count>=0.0f
      && count<=99.0f && floorf(count)==count;
    if(valid) {
      batteryCycleLimit=(unsigned int)count;
      Serial.print(F("CYCLES SET: "));Serial.println(batteryCycleLimit);
      if(!batteryCycleLimit) Serial.println(F("0 = infinite until !"));
    } else Serial.println(F("ERROR: cycles 0 (infinite) or cycles 1..99"));
    return true;
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
    Serial.println(F("SECOND PLACE: F 179.432 U -62.623 Z 61.526 (-9 mm from previous target)"));
    Serial.println(F("SECOND REPICK: F 182.412 U -67.436 Z 62.473 (reconstructed; -9 mm from previous target)"));
    Serial.println(F("SECOND RETURN: F 174.766 U -60.441 Z 38.216 (reconstructed)"));
    Serial.println(F("FIRST REPICK: F 170.289 U -72.596 Z 51.243 (reconstructed)"));
    Serial.println(F("FIRST RETURN: F 185.543 U -62.871 Z 26.442; then vertical regrasp if repeating"));
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
