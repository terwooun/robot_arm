// Reuse the software motor/serial fixtures; this is not a hardware test.
#define main xyControlSuiteMain
#define digitalWrite xyDigitalWrite
#include "xy_control_test.cpp"
#undef digitalWrite
#undef main

const int Z_PUL=8, Z_DIR=9, Z_HOME=24;
const bool Z_DIR_INVERT=false;
const int Z_ENC_SIGN=-1;
const long AS5600_CPR=4096;
const double GEAR_RATIO=32.0/9.0;
long zEncoderCount=0;
bool zEncoderOK=true, zHomed=false;
double simulatedZ=0;
int zPulses=0, zDirection=LOW, zPulse=LOW;
bool stopZ=false, failZ=false, failAtSecond=false, failAtFirstReturn=false;
bool zHomeCancelled=false, homeFailure=false;
int homeCalls=0;
double zResponse=1;
std::vector<unsigned long> zTimes;
std::vector<std::size_t> xyAtZ;
std::vector<float> heightAtZ;
int stopAfterGripperEvents=10;
std::vector<int> gripperEvents;
std::vector<float> gripperZ;
std::vector<int> gripperZPulses;
std::vector<XYMotion::Point> gripperPositions;
struct GripperMock { int angle=20; void write(int n){angle=n;gripperEvents.push_back(n);gripperZPulses.push_back(zPulses);gripperZ.push_back(zEncoderCount*(float)Z_ENC_SIGN*360.0f/AS5600_CPR/GEAR_RATIO);gripperPositions.push_back(XYMotion::forward(xyMeasuredPose()));if(gripperEvents.size()==(std::size_t)stopAfterGripperEvents) Serial.input+='!';} int read(){return angle;} } gripper;
const int OPEN_ANGLE=100, CLOSE_ANGLE=20;
int digitalRead(int) { return LOW; }
void digitalWrite(int pin,int value) {
  if(pin==Z_DIR) { zDirection=value; return; }
  if(pin==Z_PUL) {
    if(value==HIGH && zPulse==LOW) {
      ++zPulses;
      zTimes.push_back(clockUs);xyAtZ.push_back(edges.size());
      heightAtZ.push_back(XYMotion::forward(xyMeasuredPose()).up);
      simulatedZ+=(zDirection==HIGH ? -1 : 1)*4096.0/3200*zResponse;
      zEncoderCount=std::lround(simulatedZ);
      if(stopZ) Serial.input+='!';
      if((failAtFirstReturn && Serial.output.str().find("RUN: FIRST RETURN")!=std::string::npos) || failZ || (failAtSecond && Serial.output.str().find("RUN: SECOND PLACE")!=std::string::npos)) zEncoderOK=false;
    }
    zPulse=value; return;
  }
  xyDigitalWrite(pin,value);
}
void homeZ() {
  ++homeCalls;
  zHomed=!homeFailure;
  zHomeCancelled=false;
  if(zHomed) {zEncoderCount=0;simulatedZ=0;zEncoderOK=true;}
}
#include "../battery_approach.h"
void fresh() {
  reset();stopAfterGripperEvents=10;
  zEncoderCount=0;simulatedZ=0;zPulses=0;zDirection=zPulse=LOW;
  zResponse=1;zTimes.clear();xyAtZ.clear();heightAtZ.clear();gripperEvents.clear();gripperZ.clear();gripperZPulses.clear();gripperPositions.clear();
  stopZ=failZ=failAtSecond=failAtFirstReturn=false;homeFailure=false;homeCalls=0;zHomeCancelled=false;zEncoderOK=true;zHomed=false;
  batteryAboveZ=28.056f;
  batteryGripAngle=-1;batteryGripChangedUs=0;
  xyOperatorStopped=false;
  batteryHandleCommand(String("s"));
}
int main() {
  fresh();
  batteryAbove(); assert(edges.empty() && zPulses==0);
  zHomed=true;
  batteryHandleCommand(String("abovez nan")); assert(batteryAboveZ==28.056f);
  batteryHandleCommand(String("abovez 46")); assert(batteryAboveZ==28.056f);
  // The stored default must work without an abovez command.
  batteryAbove();
  assert(printed("ABOVE DONE"));
  assert(gripper.angle==100);
  assert(xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_ABOVE))<=0.15f);
  assert(fabs(batteryZAngle()-28.056f)<0.25f);
  assert(zPulse==LOW);
  assert(!zTimes.empty() && xyAtZ.front()<10 && xyAtZ.back()>edges.size()/2);
  assert(zTimes.front()<edges.back().time && zTimes.back()>edges.front().time);
  for(std::size_t i=1;i<zTimes.size();++i) assert(zTimes[i]-zTimes[i-1]>=1110);
  assert(std::labs(signedPulses(X_PUL))==std::lround((BATTERY_ABOVE.b+90)*PULSE_PER_DEGREE));
  assert(std::labs(signedPulses(Y_PUL))==std::lround((BATTERY_ABOVE.a-5)*PULSE_PER_DEGREE));
  assert(zPulses==std::lround(28.056*PULSE_PER_DEGREE));
  assert(clockUs<20000000UL); // Mock wall time: one continuous move, not old sequential replay.
  assert(!batteryPathValid(XYMotion::Pose{0,5})); // Singular configuration rejected.
  auto count=edges.size(); auto zp=zPulses;
  batteryAbove(); assert(edges.size()==count && zPulses==zp); // Not replayed from target.
  const XYMotion::Point taughtPick=XYMotion::forward(BATTERY_PICK);
  assert(fabs(taughtPick.forward-182.181f)<0.002f);
  assert(fabs(taughtPick.up+70.431f)<0.002f);
  // Latest pick is a separate command, leaves Z stationary, closes only at end.
  Serial.clearOutput();
  const auto pickEdgesBegin=edges.size();
  const auto pickStarted=clockUs;
  const XYMotion::Pose pickStart=xyMeasuredPose();
  batteryHandleCommand(String("pick"));
  assert(printed("PICK DONE"));
  assert(!printed("XY MOVE:")); // No sequence of separately stopped XY moves.
  assert(printed("synchronized XY"));
  unsigned long firstX=0,lastX=0,firstY=0,lastY=0;
  for(std::size_t i=pickEdgesBegin;i<edges.size();++i) {
    const Edge &edge=edges[i];
    if(edge.pin==X_PUL) {if(!firstX)firstX=edge.time;lastX=edge.time;}
    if(edge.pin==Y_PUL) {if(!firstY)firstY=edge.time;lastY=edge.time;}
  }
  assert(firstX && firstY && firstX<lastY && firstY<lastX);
  assert(clockUs-pickStarted<10000000UL);
  assert(batteryPathValid(pickStart,BATTERY_PICK));
  assert(gripper.angle==20 && zPulses==zp);
  assert(xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_PICK))<=0.15f);
  count=edges.size();
  batteryPick();assert(edges.size()==count); // Reject repeat descent from pick pose.
  fresh();zHomed=true;
  batteryPick();assert(edges.empty()); // Cannot pick directly from start pose.
  fresh();zHomed=true;batteryAbove();
  Serial.clearOutput();stopAfterRisingEdges=edges.size()+1;
  batteryPick();assert(printed("PICK PAUSED"));assert(gripper.angle==100);
  expectIdle();
  fresh();zHomed=true;batteryAbove();
  Serial.clearOutput();zEncoderOK=false;
  count=edges.size();batteryPick();assert(edges.size()==count && gripper.angle==100);
  fresh();zHomed=true;batteryAbove();
  Serial.clearOutput();feedbackScale=0.8;
  batteryPick();
  assert(printed("PICK FINISHED WITH WARNING"));
  assert(gripper.angle==20 && xyCalibrated);
  expectIdle();
  for(int fault=1;fault<3;++fault) {
    fresh();zHomed=true;

    if(fault==1) stopZ=true;
    if(fault==2) failZ=true;
    batteryAbove();
    assert(printed("ABOVE PAUSED"));
    assert(!printed("ABOVE DONE"));
    assert(zPulse==LOW);
    assert(zPulses<=16);
    expectIdle();
  }
  fresh();
  batteryHandleCommand(String("abovez 27")); assert(batteryAboveZ==27);
  fresh(); assert(batteryAboveZ==28.056f);
  // Manual-alignment mode must advance past XY and Z following warnings.
  fresh(); zHomed=true;
  feedbackScale=0.9; zResponse=0.97;
  batteryAbove();
  assert(printed("ABOVE FINISHED WITH WARNING"));
  assert(!printed("ABOVE PAUSED"));
  assert(printed("WARNING: AUTO Z"));
  assert(zPulses>0 && xyCalibrated);
  expectIdle();
  // No encoder motion still terminates after bounded requested segments.
  fresh(); zHomed=true;
  feedbackMultiplier=0;
  batteryAbove();
  assert(printed("ABOVE FINISHED WITH WARNING"));
  assert(zPulses>0 && edges.size()<10000);
  // An operator stop promptly stops all synchronized axes.
  fresh(); zHomed=true;  stopAfterRisingEdges=1;
  batteryAbove();
  assert(printed("ABOVE PAUSED") && zPulses<=1);
  expectIdle();
  // Complete ordered transport, release, and no automatic retreat.
  fresh();zHomed=true;batteryAbove();batteryPick();
  Serial.clearOutput();
  const auto carryZStart=heightAtZ.size();
  const auto placeStarted=clockUs;
  batteryPlace();
  assert(batteryStageCompleted && printed("PLACE DONE"));
  assert(gripper.angle==100);
  assert(fabs(batteryZAngle()-BATTERY_PLACE_Z)<0.25f);
  assert(xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_PLACE))<=0.15f);
  for(std::size_t i=carryZStart;i<heightAtZ.size();++i)
    assert(fabs(heightAtZ[i]-BATTERY_CARRY_UP)<0.3f);
  const auto output=Serial.output.str();
  const auto liftText=output.substr(output.find("PLACE: LIFT"),output.find("PLACE: ROTATE")-output.find("PLACE: LIFT"));
  assert(liftText.find("XY MOVE:")==liftText.rfind("XY MOVE:"));
  assert(liftText.find("XY MOVE:")!=std::string::npos); // Exactly one >50mm lift.
  const auto lowerText=output.substr(output.find("PLACE: LOWER"));
  assert(lowerText.find("XY MOVE:")==lowerText.rfind("XY MOVE:"));
  assert(lowerText.find("XY MOVE:")!=std::string::npos);
  assert(clockUs-placeStarted<22000000UL);

  assert(output.find("PLACE: LIFT")<output.find("PLACE: ROTATE"));
  assert(output.find("PLACE: ROTATE")<output.find("PLACE: TRANSFER"));
  assert(output.find("PLACE: TRANSFER")<output.find("PLACE: LOWER"));
  count=edges.size();zp=zPulses;batteryPlace();
  assert(edges.size()==count && zPulses==zp && !batteryStageCompleted);
  // Stop while lifting: no rotation and no release.
  fresh();zHomed=true;batteryAbove();batteryPick();
  Serial.clearOutput();zp=zPulses;stopAfterRisingEdges=edges.size()+1;
  batteryPlace();
  assert(printed("PLACE PAUSED") && !batteryStageCompleted);
  assert(zPulses==zp && gripper.angle==20);expectIdle();
  // A rotation sensor failure also leaves the battery held.
  fresh();zHomed=true;batteryAbove();batteryPick();
  Serial.clearOutput();failZ=true;
  batteryPlace();
  assert(printed("PLACE PAUSED") && gripper.angle==20);
  assert(!printed("PLACE: LOWER"));expectIdle();
  // One command completes a cycle; no repeat loop or stale success on failure.
  fresh();zHomed=true;Serial.clearOutput();batteryRun();
  assert(printed("RUN CYCLE DONE") && gripper.angle==100);
  assert(homeCalls==1);
  assert(xyEndpointError(xyMeasuredPose(),BATTERY_FIRST_RETURN)<=0.15f);
  assert(fabs(batteryZAngle()-26.442f)<0.25f);
  const auto runText=Serial.output.str();
  assert(runText.find("RUN: SECOND PICK")<runText.find("RUN: SECOND PLACE"));
  assert(runText.find("RUN: SECOND PLACE")<runText.find("RUN: SECOND REPICK"));
  assert(runText.find("RUN: SECOND REPICK")<runText.find("RUN: SECOND RETURN"));
  // Two full cycles in one call, then ! at the final release.
  fresh();stopAfterGripperEvents=18;Serial.clearOutput();batteryRun();
  assert(homeCalls==1 && gripperEvents.size()==18 && printed("RUN STOPPED by !"));
  assert(!printed("RETURN ABOVE START") && printed("RUN: REPEAT FIRST PICK"));
  assert(fabs(gripperPositions[10].forward-(BATTERY_FIRST_RETURN.forward+0.8f))<0.15f);
  assert(fabs(gripperZ[10]-26.842f)<0.25f);
  assert(fabs(gripperPositions[12].forward-(BATTERY_SECOND_RETURN.forward+0.8f))<0.15f);
  assert(fabs(gripperPositions[12].up-(BATTERY_SECOND_RETURN.up-1.0f))<0.15f);
  expectIdle();
  fresh();homeFailure=true;Serial.clearOutput();batteryRun();
  assert(printed("RUN PAUSED at homez") && edges.empty() && homeCalls==1);
  // No separate homez command is required. Fresh default starts unhomed.
  fresh();Serial.clearOutput();batteryRun();
  assert(printed("RUN CYCLE DONE") && homeCalls==1);
  assert((gripperEvents==std::vector<int>{100,100,20,100,20,100,20,100,20,100}));
  // Check actual commanded grip/release endpoints, including the shifted regrasp.
  const XYMotion::Point expectedSecond[]={{174.613f,-72.974f},{181.032f,-62.623f},{182.412f,-67.436f},{174.766f,-60.441f}};
  for(int i=0;i<4;++i) {
    assert(fabs(gripperPositions[i+4].forward-expectedSecond[i].forward)<0.15f);
    assert(fabs(gripperPositions[i+4].up-expectedSecond[i].up)<0.15f);
  }
  assert(xyEndpointError(xyMeasuredPose(),BATTERY_FIRST_RETURN)<=0.15f);
  assert(fabs(gripperPositions[8].forward-170.289f)<0.15f);
  assert(fabs(gripperPositions[8].up+72.596f)<0.15f);
  assert(fabs(gripperPositions[9].forward-185.543f)<0.15f);
  assert(fabs(gripperPositions[9].up+62.871f)<0.15f);
  assert(fabs(gripperZ[6]-61.526f)<0.25f);
  assert(gripperZPulses[5]==gripperZPulses[6]); // No Z pulses from release to regrasp.
  assert(fabs(gripperZ[8]-51.243f)<0.25f);
  assert(fabs(gripperZ[9]-26.442f)<0.25f);
  // ! after repeat pickup retains the closed gripper, skips the next transfer.
  fresh();stopAfterGripperEvents=11;Serial.clearOutput();batteryRun();
  assert(gripper.angle==20 && gripperEvents.size()==11 && !batteryStageCompleted);
  expectIdle();
  // Third-cycle offset remains +0.4, not +0.8; home remains initial-only.
  fresh();stopAfterGripperEvents=26;Serial.clearOutput();batteryRun();
  assert(homeCalls==1 && gripperEvents.size()==26);
  assert(fabs(gripperZ[18]-26.842f)<0.25f);
  assert(fabs(gripperZ[20]-38.616f)<0.25f);
  assert(fabs(gripperZ[25]-26.842f)<0.25f);
  assert(gripperZPulses[13]==gripperZPulses[14]);
  assert(gripperZPulses[21]==gripperZPulses[22]);
  assert(fabs(gripperPositions[18].forward-(BATTERY_FIRST_RETURN.forward+0.8f))<0.15f);
  assert(fabs(gripperPositions[21].forward-181.832f)<0.15f);
  assert(fabs(gripperPositions[25].forward-(BATTERY_FIRST_RETURN.forward+0.8f))<0.15f);
  // Every grip/release target in cycles two and three has the same fixed XY shift.
  const XYMotion::Point repeatBase[]={BATTERY_FIRST_RETURN,XYMotion::forward(BATTERY_PLACE),
    BATTERY_SECOND_RETURN,BATTERY_SECOND_PLACE,BATTERY_SECOND_REPICK,BATTERY_SECOND_RETURN,
    BATTERY_FIRST_REPICK,BATTERY_FIRST_RETURN};
  for(int cycle=0;cycle<2;++cycle) for(int station=0;station<8;++station) {
    const XYMotion::Point &actual=gripperPositions[10+cycle*8+station];
    assert(fabs(actual.forward-(repeatBase[station].forward+0.8f))<0.15f);
    assert(fabs(actual.up-(repeatBase[station].up-1.0f))<0.15f);
  }
  expectIdle();
  // Identical straight path: open gripper is faster; closed/intermediate stays loaded.
  fresh();zHomed=true;gripper.angle=OPEN_ANGLE;
  const XYMotion::Point speedOrigin=XYMotion::forward(xyMeasuredPose());
  const XYMotion::Point speedTarget={speedOrigin.forward,speedOrigin.up+20};
  auto speedStarted=clockUs;assert(batteryStraightMove(speedTarget));
  const auto emptyTime=clockUs-speedStarted;
  assert(batterySpeedScale()==6.0f);
  fresh();zHomed=true;gripper.angle=CLOSE_ANGLE;
  speedStarted=clockUs;assert(batteryStraightMove(speedTarget));
  const auto loadedTime=clockUs-speedStarted;
  assert(emptyTime<loadedTime*0.8 && batterySpeedScale()==2.0f);
  gripper.angle=60;assert(batterySpeedScale()==2.0f);
  fresh();zHomed=true;gripper.angle=OPEN_ANGLE;stopAfterRisingEdges=1;
  assert(!batteryStraightMove(speedTarget));expectIdle();
  // Pure Z motion: open-gripper profile is faster, closed/intermediate use 2x.
  fresh();zHomed=true;gripper.angle=OPEN_ANGLE;
  auto zStarted=clockUs;
  assert(batterySynchronizedMove(xyMeasuredPose(),xyMeasuredPose(),true,5));
  const auto emptyZTime=clockUs-zStarted;
  assert(batteryZSpeedScale()==6.0f);
  fresh();zHomed=true;gripper.angle=CLOSE_ANGLE;zStarted=clockUs;
  assert(batterySynchronizedMove(xyMeasuredPose(),xyMeasuredPose(),true,5));
  assert(emptyZTime<clockUs-zStarted && batteryZSpeedScale()==4.0f);
  for(std::size_t i=1;i<zTimes.size();++i) assert(zTimes[i]-zTimes[i-1]>=1666);
  // Equal-joint XY move: empty DDA ticks stay constant, loaded move retains ramp.
  unsigned long emptyFirst=0,emptyMiddle=0,loadedFirst=0,loadedMiddle=0;
  for(int loaded=0;loaded<2;++loaded) {
    fresh();zHomed=true;gripper.angle=loaded ? CLOSE_ANGLE:OPEN_ANGLE;
    const auto startPose=xyMeasuredPose();
    const XYMotion::Pose goal={startPose.a+5,startPose.b+5};
    assert(batterySynchronizedMove(startPose,goal,false));
    std::vector<unsigned long> xTimes;
    for(const auto &edge:edges) if(edge.pin==X_PUL) xTimes.push_back(edge.time);
    assert(xTimes.size()>=158);
    const auto first=xTimes[2]-xTimes[1],middle=xTimes[80]-xTimes[79];
    if(loaded) {loadedFirst=first;loadedMiddle=middle;}
    else {emptyFirst=first;emptyMiddle=middle;}
  }
  assert(std::labs((long)emptyFirst-(long)emptyMiddle)<=200);
  assert(loadedFirst>loadedMiddle);
  // The straight Cartesian planner also bypasses acceleration only when requested.
  fresh();gripper.angle=OPEN_ANGLE;
  auto profileStart=clockUs;xyMove(0,10,false,true,1.0f,true);
  assert(xyLastMoveCompleted);const auto constantTime=clockUs-profileStart;
  fresh();profileStart=clockUs;xyMove(0,10,false,true,1.0f,false);
  assert(xyLastMoveCompleted && clockUs-profileStart>constantTime);
  // Grip wait credits elapsed time and does not restart for the same command.
  fresh();batteryCommandGrip(OPEN_ANGLE);
  auto openWaitStart=clockUs;assert(batteryAwaitGrip());
  assert(clockUs-openWaitStart>=149000UL && clockUs-openWaitStart<151000UL);
  fresh();batteryCommandGrip(OPEN_ANGLE);clockUs+=500000UL;
  auto waitStart=clockUs;assert(batteryAwaitGrip());assert(clockUs-waitStart<1000UL);
  batteryCommandGrip(OPEN_ANGLE);waitStart=clockUs;
  assert(batteryAwaitGrip() && clockUs-waitStart<1000UL);
  batteryCommandGrip(CLOSE_ANGLE);waitStart=clockUs;
  assert(batteryAwaitGrip() && clockUs-waitStart>=199000UL);
  batteryCommandGrip(OPEN_ANGLE);Serial.input="!";assert(!batteryAwaitGrip());
  // Stop at run entry is checked before homing or any servo/motor movement.
  fresh();Serial.input="!";batteryRun();
  assert(homeCalls==0 && edges.empty() && gripperEvents.empty());
  // Later stage sensor failure must not release or advance to repick/return.
  fresh();failAtSecond=true;Serial.clearOutput();batteryRun();
  assert(printed("RUN PAUSED at second place") && gripper.angle==20);
  assert(!printed("RUN: SECOND REPICK"));expectIdle();
  fresh();failAtFirstReturn=true;Serial.clearOutput();batteryRun();
  assert(printed("RUN PAUSED at first return") && gripper.angle==20);
  assert(!printed("RUN: EMPTY LIFT") && !batteryStageCompleted);expectIdle();
  std::cout<<"Battery approach: prerequisites, path, endpoint, warning, stop, sensor tests passed\n";
}
