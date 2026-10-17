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
struct GripperMock { int angle=20; void write(int n){angle=n;} } gripper;
const int OPEN_ANGLE=100;
int digitalRead(int) { return LOW; }
void digitalWrite(int pin,int value) {
  if(pin==Z_DIR) { zDirection=value; return; }
  if(pin==Z_PUL) {
    if(value==HIGH && zPulse==LOW) {
      ++zPulses;
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
  zResponse=1;
  stopZ=failZ=false;zEncoderOK=true;zHomed=false;
  batteryZConfirmed=false;batteryAboveZ=31.097f;
  xyOperatorStopped=false;
  batteryHandleCommand(String("startpose"));
}
int main() {
  fresh();
  batteryAbove(); assert(edges.empty() && zPulses==0);
  zHomed=true;
  batteryAbove(); assert(edges.empty() && zPulses==0);
  batteryHandleCommand(String("abovez nan")); assert(!batteryZConfirmed);
  batteryHandleCommand(String("abovez 46")); assert(!batteryZConfirmed);
  batteryHandleCommand(String("abovez 31.097"));
  batteryAbove();
  assert(printed("ABOVE DONE"));
  assert(gripper.angle==100);
  assert(xyEndpointError(xyMeasuredPose(),XYMotion::forward(BATTERY_ABOVE))<=0.15f);
  assert(fabs(batteryZAngle()-31.097f)<0.25f);
  assert(zPulse==LOW);
  auto count=edges.size(); auto zp=zPulses;
  batteryAbove(); assert(edges.size()==count && zPulses==zp); // Not replayed from target.
  for(int fault=1;fault<3;++fault) {
    fresh();zHomed=true;batteryZConfirmed=true;

    if(fault==1) stopZ=true;
    if(fault==2) failZ=true;
    batteryAbove();
    assert(printed("ABOVE PAUSED"));
    assert(!printed("ABOVE DONE"));
    assert(zPulse==LOW);
    assert(zPulses<=1);
    expectIdle();
  }
  // Manual-alignment mode must advance past XY and Z following warnings.
  fresh(); zHomed=true; batteryZConfirmed=true;
  feedbackScale=0.9; zResponse=0.97;
  batteryAbove();
  assert(printed("ABOVE FINISHED WITH WARNING"));
  assert(!printed("ABOVE PAUSED"));
  assert(printed("WARNING: ABOVE Z"));
  assert(zPulses>0 && xyCalibrated);
  expectIdle();
  // No encoder motion still terminates after bounded requested segments.
  fresh(); zHomed=true; batteryZConfirmed=true;
  feedbackMultiplier=0;
  batteryAbove();
  assert(printed("ABOVE FINISHED WITH WARNING"));
  assert(zPulses>0 && edges.size()<10000);
  // An operator stop during the first XY leg never rotates Z.
  fresh(); zHomed=true; batteryZConfirmed=true; stopAfterRisingEdges=1;
  batteryAbove();
  assert(printed("ABOVE PAUSED") && zPulses==0);
  expectIdle();
  std::cout<<"Battery approach: prerequisites, path, endpoint, warning, stop, sensor tests passed\n";
}
