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
bool stopZ=false, failZ=false;
double zResponse=1;
std::vector<unsigned long> zTimes;
std::vector<std::size_t> xyAtZ;
std::vector<float> heightAtZ;
struct GripperMock { int angle=20; void write(int n){angle=n;} int read(){return angle;} } gripper;
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
      if(failZ) zEncoderOK=false;
    }
    zPulse=value; return;
  }
  xyDigitalWrite(pin,value);
}
#include "../battery_approach.h"
void fresh() {
  reset();
  zEncoderCount=0;simulatedZ=0;zPulses=0;zDirection=zPulse=LOW;
  zResponse=1;zTimes.clear();xyAtZ.clear();heightAtZ.clear();
  stopZ=failZ=false;zEncoderOK=true;zHomed=false;
  batteryAboveZ=28.056f;
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
  for(std::size_t i=1;i<zTimes.size();++i) assert(zTimes[i]-zTimes[i-1]>=8000);
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
  assert(printed("RUN DONE") && gripper.angle==100);
  assert(xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_PLACE))<=0.15f);
  count=edges.size();zp=zPulses;batteryRun();
  assert(printed("RUN PAUSED at above") && edges.size()==count && zPulses==zp);
  fresh();Serial.clearOutput();batteryRun();
  assert(printed("RUN PAUSED at above") && edges.empty());
  std::cout<<"Battery approach: prerequisites, path, endpoint, warning, stop, sensor tests passed\n";
}
