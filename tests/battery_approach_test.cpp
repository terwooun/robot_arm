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
struct GripperMock { int angle=20; void write(int n){angle=n;} } gripper;
const int OPEN_ANGLE=100;
int digitalRead(int) { return LOW; }
void digitalWrite(int pin,int value) {
  if(pin==Z_DIR) { zDirection=value; return; }
  if(pin==Z_PUL) {
    if(value==HIGH && zPulse==LOW) {
      ++zPulses;
      zTimes.push_back(clockUs);xyAtZ.push_back(edges.size());
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
  zResponse=1;zTimes.clear();xyAtZ.clear();
  stopZ=failZ=false;zEncoderOK=true;zHomed=false;
  batteryAboveZ=28.056f;
  xyOperatorStopped=false;
  batteryHandleCommand(String("startpose"));
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
  assert(printed("WARNING: ABOVE Z"));
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
  std::cout<<"Battery approach: prerequisites, path, endpoint, warning, stop, sensor tests passed\n";
}
